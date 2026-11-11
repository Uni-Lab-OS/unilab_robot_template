"""Explicit vendor SDK workers. SDK installation and commissioning stay external.

Each blocking vendor call runs behind a bounded process boundary. Killing that
worker is transport quarantine, never evidence that a physical robot stopped.
Only observed vendor feedback produces a joint value; no zero-state fallback.
"""
from __future__ import annotations

import ctypes
import hashlib
import importlib
import json
import math
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

if __package__:
    from .hardware_driver import DriverProtocolError, DriverTimeout, _numbers
else:
    # The SDK worker may run under ROS Noetic's Python 3.8 while the installed
    # brand host uses 3.11+. No brand package or asset checkout is imported there.
    class DriverProtocolError(RuntimeError): pass
    class DriverTimeout(DriverProtocolError): pass
    def _numbers(values,count):
        result=tuple(float(v) for v in values)
        if len(result)!=count or not all(math.isfinite(v) for v in result):
            raise ValueError('Invalid vendor coordinates')
        return result


class NativeSDKAdapter:
    def __init__(self, family, mapping, *, python_executable, deployment_evidence,
                 config, timeout_s=15, targets=None, offline=False):
        if not deployment_evidence or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Explicit deployment evidence and positive timeout required')
        self.family=family; self.mapping=mapping; self.config=dict(config,loopback_only=offline)
        self.python=python_executable; self.timeout=timeout_s; self.offline=offline
        self.targets=targets or {}; self.receipts={}; self.transcript=[]
        self.process=None; self.failed=False; self.connected=False
        self.lock=threading.Lock(); self.replies=queue.Queue(); self.counter=0
        self.id_lock=threading.Lock();self.sent_ids=set()

    def _reserve_id(self,command_id):
        with self.id_lock:
            if not isinstance(command_id,str) or not command_id.strip() or command_id in self.sent_ids:
                raise ValueError('Nonempty unique command_id required; no automatic replay')
            self.sent_ids.add(command_id)

    def _call(self, operation, **arguments):
        with self.lock:
            if self.failed:
                raise DriverProtocolError('SDK worker quarantined after an earlier failure')
            if self.process is None:
                command=[self.python,'-I','-B','-u','-m','unilab_robot_runtime.native_sdk']
                if self.config.get('worker_script'):
                    worker=Path(self.config['worker_script']).resolve()
                    if hashlib.sha256(worker.read_bytes()).hexdigest()!=self.config.get('worker_sha256'):
                        raise ValueError('External SDK worker digest mismatch')
                    command=[self.python,'-I','-B','-u',str(worker)]
                self.process=subprocess.Popen(command,stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,encoding='utf-8',
                    creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
                def read(stream, replies=False):
                    for line in stream:
                        self.transcript.append(line.rstrip())
                        if replies and line.startswith('UNILAB_SDK_REPLY '):
                            self.replies.put(json.loads(line[len('UNILAB_SDK_REPLY '):]))
                threading.Thread(target=read,args=(self.process.stdout,True),daemon=True).start()
                threading.Thread(target=read,args=(self.process.stderr,),daemon=True).start()
            self.counter+=1
            request=dict(id=self.counter,operation=operation,family=self.family,
                         config=self.config,arguments=arguments)
            self.transcript.append(request)
            try:
                self.process.stdin.write(json.dumps(request)+'\n'); self.process.stdin.flush()
                reply=self.replies.get(timeout=self.timeout)
                if reply.get('id')!=self.counter:
                    raise DriverProtocolError('Uncorrelated SDK response')
                if not reply.get('ok'):
                    raise DriverProtocolError(reply.get('error','Vendor call failed'))
                return reply.get('result')
            except queue.Empty as exc:
                self.failed=True; self.close()
                raise DriverTimeout('Vendor call deadline exceeded; worker killed, robot stop unknown') from exc
            except Exception:
                self.failed=True; self.close(); raise

    def connect(self, host, port=None):
        if self.offline and host not in ('127.0.0.1','::1'):
            raise ValueError('Offline SDK validation requires a loopback address')
        result=self._call('connect',host=host,port=port)
        self.connected=True
        return result

    def read_joints(self):
        if not self.connected: raise DriverProtocolError('SDK not connected')
        result=self._call('read_joints')
        values=_numbers(result['values'],self.mapping.dof)
        if isinstance(result['units'],list):
            if result['units']!=list(self.mapping.native_units): raise DriverProtocolError('SDK and calibrated per-axis units differ')
            return self.mapping.from_native(values)
        if result['units']=='rad': values=tuple(math.degrees(v) for v in values)
        elif result['units']!='deg': raise DriverProtocolError('Unresolved SDK joint units')
        return self.mapping.from_degrees(values)

    def move_joint(self, q, *, command_id, **motion):
        if not self.connected: raise DriverProtocolError('SDK not connected')
        degrees=self.mapping.to_native(q)
        self._reserve_id(command_id)
        ack=self._call('move_joint',degrees=degrees,**motion)
        # This only means a request was encoded/accepted. Vendor motion status
        # and path execution are not uniformly qualified across these SDKs.
        receipt={'command_id':command_id,'state':'submitted','completed':False,
                 'hardware_verified':False,'vendor_result':ack,
                 'scope':'vendor_sdk_loopback' if self.offline else 'vendor_sdk_request'}
        self.receipts[command_id]=receipt
        return receipt

    def execute_target(self,target_ref,parameters,command_id):
        if target_ref not in self.targets: raise ValueError('Target is not deployment-allowlisted')
        q=self.targets[target_ref]
        resolved=parameters.get('resolved_target',{})
        if resolved.get('type')!='joint_positions' or tuple(resolved.get('joint_positions_si',()))!=tuple(q):
            raise ValueError('Resolved target differs from deployment target')
        options={k:v for k,v in parameters.items() if k!='resolved_target'}
        return self.move_joint(q,command_id=command_id,**options)

    def query_command(self,command_id): return self.receipts.get(command_id)
    def move_cartesian(self, waypoints, *, command_id):
        if not self.connected or self.family!='inovo_commander':
            raise DriverProtocolError('Cartesian SDK entry is only available for connected Inovo')
        self._reserve_id(command_id)
        result=self._call('move_cartesian',waypoints=waypoints)
        receipt={'command_id':command_id,'state':'sdk_reported_success','completed':False,
                 'hardware_verified':False,'vendor_result':result}
        self.receipts[command_id]=receipt
        return receipt
    def request_stop(self,command_id,reason): return False
    def close(self):
        self.connected=False
        if self.process is not None and self.process.poll() is None:
            self.process.kill(); self.process.wait(timeout=5)


def _module(name, config):
    module=importlib.import_module(name)
    path=Path(module.__file__).resolve()
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    expected=config.get('module_sha256',{}).get(name)
    if not expected or expected!=digest:
        raise ValueError('Explicit matching installed SDK module hash required: '+name)
    for relative,expected in config.get('native_binary_sha256',{}).items():
        binary=(path.parent/relative).resolve()
        if path.parent not in binary.parents or hashlib.sha256(binary.read_bytes()).hexdigest()!=expected:
            raise ValueError('Vendor native binary digest mismatch')
    return module


class VendorSession:
    """Calls real SDK methods; HIWIN uses the disclosed transport-check patch."""
    def __init__(self,family,config):
        self.family=family; self.config=config; self.client=None
        for directory in config.get('sdk_search_paths',[]):
            path=Path(directory).resolve()
            if not path.is_dir() or 'uni-lab-assets' in path.parts:
                raise ValueError('Explicit installed SDK search path required')
            sys.path.insert(0,str(path))
        self.dof=int(config['dof'])
        if not 1<=self.dof<=7: raise ValueError('Invalid DOF')

    def connect(self,host,port=None):
        c=self.config; f=self.family
        if f=='elephant280':
            m=_module('pymycobot.mycobot280socket',c)
            self.client=m.MyCobot280Socket(host,netport=port or 9000)
        elif f=='elephant_pro600_local':
            if host not in ('127.0.0.1','::1') or self.dof!=6 or not c.get('pro600_controller_evidence'):
                raise ValueError('Pro600 requires explicit local six-axis controller binding')
            m=_module('pymycobot.mycobotpro630',c)
            if m.Robots.PRO_600.value!=502: raise ValueError('Unexpected Pro600 SDK model enum')
            # Controller-local LinuxCNC only. Never initialize HAL/CAN, power or
            # robot configuration automatically; this boundary is read-only.
            self.client=m.Phoenix(do_init=False)
        elif f=='igus_cri':
            m=_module('cri_lib.cri_controller',c)
            self.client=m.CRIController(); self.client.DEFAULT_ANSWER_TIMEOUT=c.get('answer_timeout_s',2)
            self.client.connect(host,port or 3920)
            self.client.wait_for_status_update(timeout=c.get('answer_timeout_s',2))
        elif f=='elite_ec':
            m=_module('elite._ec',c)
            self.client=m._EC(host,auto_connect=False)
            status=self.client.connect_ETController(host,port or 8055)
            if not status or status[0] is not True: raise RuntimeError('Elite EC connection failed')
        elif f=='agilebot_a':
            m=_module('Agilebot.IR.A.arm',c)
            self.client=m.Arm(local_proxy=False)
            status=self.client.connect(host,teach_panel_ip=host)
            from Agilebot.IR.A.status_code import StatusCodeEnum
            if status!=StatusCodeEnum.OK: raise RuntimeError(str(status))
            if self.client.model!=c['expected_model']:
                raise RuntimeError('Controller model does not match deployment binding')
        elif f=='estun_eri':
            p=Path(c['bridge']).resolve()
            if hashlib.sha256(p.read_bytes()).hexdigest()!=c['bridge_sha256']:
                raise ValueError('ESTUN bridge digest mismatch')
            lib=ctypes.CDLL(str(p)); ptr=ctypes.c_void_p; integer=ctypes.c_int
            signatures={'create':([],ptr),'destroy':([ptr],integer),
                'configure':([ptr,ctypes.c_char_p,ctypes.c_ushort,ctypes.c_ushort,ctypes.c_ushort],integer),
                'connection':([ptr,ctypes.POINTER(ctypes.c_ubyte)],integer),
                'joints':([ptr,ctypes.POINTER(ctypes.c_double)],integer),
                'movj':([ptr,ctypes.POINTER(ctypes.c_double)],integer)}
            for name,(args,result) in signatures.items():
                fun=getattr(lib,'unilab_estun_'+name); fun.argtypes=args; fun.restype=result
            self.client=lib; self.handle=lib.unilab_estun_create()
            if not self.handle: raise RuntimeError('ESTUN create failed')
            rc=lib.unilab_estun_configure(self.handle,host.encode(),*c['ports'])
            channels=(ctypes.c_ubyte*3)(); status=lib.unilab_estun_connection(self.handle,channels)
            deadline=time.monotonic()+float(c.get('connection_timeout_s',2))
            while rc==1 and status==0 and list(channels)!=[1,1,1] and time.monotonic()<deadline:
                time.sleep(.05);status=lib.unilab_estun_connection(self.handle,channels)
            if rc!=1 or status!=0 or list(channels)!=[1,1,1]:
                raise RuntimeError(f'ESTUN configure={rc}, channels={list(channels)}; parameters accepted does not mean connected')
        elif f=='hiwin_commander':
            if self.dof!=6: raise ValueError('HIWIN bridge requires six axes')
            p=Path(c['bridge']).resolve()
            if hashlib.sha256(p.read_bytes()).hexdigest()!=c['bridge_sha256']:
                raise ValueError('HIWIN bridge digest mismatch')
            lib=ctypes.CDLL(str(p)); ptr=ctypes.c_void_p; dp=ctypes.POINTER(ctypes.c_double)
            signatures={'create':([ctypes.c_char_p,ctypes.c_int],ptr),
                'connect':([ptr],ctypes.c_int),'destroy':([ptr],ctypes.c_int),
                'read_joints':([ptr,dp],ctypes.c_int),'move_joints':([ptr,dp],ctypes.c_int),
                'error':([],ctypes.c_char_p)}
            for name,(args,result) in signatures.items():
                fun=getattr(lib,'unilab_hiwin_'+name);fun.argtypes=args;fun.restype=result
            self.client=lib;self.handle=lib.unilab_hiwin_create(host.encode(),port or 1503)
            if not self.handle: raise RuntimeError('HIWIN create failed')
            rc=lib.unilab_hiwin_connect(self.handle)
            if rc!=0: raise RuntimeError('HIWIN connect: '+str(rc)+' '+lib.unilab_hiwin_error().decode(errors='replace'))
        elif f=='inovo_commander':
            m=_module('commander_api.motion_control_client',c)
            import rospy
            import os
            # A ROS master and matching message packages are explicit runtime
            # dependencies, not silently inherited from an unrelated workspace.
            if os.environ.get('ROS_MASTER_URI')!=c['ros_master_uri']:
                raise ValueError('ROS_MASTER_URI differs from pinned deployment')
            if not rospy.core.is_initialized(): rospy.init_node('unilab_inovo_sdk',anonymous=True,disable_signals=True)
            self.client=m.MotionControlClient(c['namespace'])
        elif f=='flexiv_rdk_1_9':
            m=_module('flexivrdk',c)
            if not c.get('native_binary_sha256'): raise ValueError('Pinned Flexiv native extension required')
            if not c.get('network_interfaces'): raise ValueError('Explicit Flexiv interface whitelist required')
            if c.get('loopback_only') and c['network_interfaces']!=['127.0.0.1']:
                raise ValueError('Offline Flexiv interface must be loopback')
            self.client=m.Robot(c['robot_serial'],network_interface_whitelist=c['network_interfaces'])
        elif f=='franka_panda_0_9':
            if self.dof!=7: raise ValueError('Panda bridge requires seven axes')
            p=Path(c['bridge']).resolve()
            if hashlib.sha256(p.read_bytes()).hexdigest()!=c['bridge_sha256']:
                raise ValueError('Panda 0.9 bridge digest mismatch')
            lib=ctypes.CDLL(str(p));ptr=ctypes.c_void_p
            signatures={'create':([],ptr),'connect':([ptr,ctypes.c_char_p],ctypes.c_int),
                'read_once':([ptr,ctypes.POINTER(ctypes.c_double)],ctypes.c_int),
                'server_version':([ptr,ctypes.POINTER(ctypes.c_int)],ctypes.c_int),
                'destroy':([ptr],ctypes.c_int),'error':([],ctypes.c_char_p)}
            for name,(args,result) in signatures.items():
                fun=getattr(lib,'unilab_panda_'+name);fun.argtypes=args;fun.restype=result
            self.client=lib;self.handle=lib.unilab_panda_create()
            if not self.handle: raise RuntimeError('Panda create failed')
            rc=lib.unilab_panda_connect(self.handle,host.encode())
            if rc!=0: raise RuntimeError('Panda connect: '+str(rc)+' '+lib.unilab_panda_error().decode(errors='replace'))
        else:
            raise ValueError('No qualified native entry for family '+f)
        return {'transport_initialized':True,'hardware_verified':False}

    def read_joints(self):
        f=self.family; c=self.client
        if f in ('elephant280','elephant_pro600_local'):
            values=c.get_angles()
            if not isinstance(values,(list,tuple)): raise RuntimeError('myCobot get_angles failed/timeout: '+str(values))
        elif f=='igus_cri':
            c.wait_for_status_update(timeout=self.config.get('answer_timeout_s',2))
            with c.robot_state_lock:
                if not c.robot_state.category_time_ns.get('STATUS'): raise RuntimeError('No received joint state')
                values=[getattr(c.robot_state.joints_current,'A'+str(i+1)) for i in range(self.dof)]
            if isinstance(self.config.get('joint_units'),list): return {'values':list(_numbers(values,self.dof)),'units':self.config['joint_units']}
        elif f=='elite_ec':
            values=c.get_joint()
        elif f=='agilebot_a':
            from Agilebot.IR.A.status_code import StatusCodeEnum
            pose,status=c.motion.get_current_pose()
            if status!=StatusCodeEnum.OK: raise RuntimeError(str(status))
            values=[getattr(pose.joint,'j'+str(i+1)) for i in range(self.dof)]
        elif f=='estun_eri':
            data=(ctypes.c_double*16)(); rc=c.unilab_estun_joints(self.handle,data)
            if rc!=1: raise RuntimeError('ESTUN getJointValue failed/timeout: '+str(rc))
            units=self.config.get('joint_units')
            if not (units in ('rad','deg') or isinstance(units,list) and len(units)==self.dof and all(u in ('rad','deg','mm','m') for u in units)): raise ValueError('ESTUN units require controller documentation')
            return {'values':list(data)[:self.dof],'units':self.config['joint_units']}
        elif f=='hiwin_commander':
            data=(ctypes.c_double*6)();rc=c.unilab_hiwin_read_joints(self.handle,data)
            if rc!=0: raise RuntimeError('HIWIN read: '+str(rc)+' '+c.unilab_hiwin_error().decode(errors='replace'))
            return {'values':list(data),'units':'rad'}
        elif f=='inovo_commander':
            message=c.get_joint_angles()
            names=self.config['joint_names']
            if len(names)!=self.dof or len(set(names))!=len(names) or len(set(message.name))!=len(message.name):
                raise ValueError('Invalid Inovo joint-name binding')
            import rospy
            max_age=float(self.config['max_state_age_s'])
            age=(rospy.Time.now()-message.header.stamp).to_sec()
            if not math.isfinite(max_age) or max_age<=0 or message.header.stamp.to_sec()<=0 or not 0<=age<=max_age:
                raise RuntimeError('Inovo joint feedback is stale or has no valid timestamp')
            if len(message.name)!=len(message.position): raise ValueError('Inovo name/position length mismatch')
            state=dict(zip(message.name,message.position))
            values=[state[name] for name in names]
            return {'values':list(_numbers(values,self.dof)),'units':'rad'}
        elif f=='flexiv_rdk_1_9':
            if not c.connected(): raise RuntimeError('Flexiv connection lost')
            return {'values':list(_numbers(c.states().q,self.dof)),'units':'rad'}
        elif f=='franka_panda_0_9':
            data=(ctypes.c_double*7)();rc=c.unilab_panda_read_once(self.handle,data)
            if rc!=0: raise RuntimeError('Panda read_once: '+str(rc)+' '+c.unilab_panda_error().decode(errors='replace'))
            return {'values':list(_numbers(data,7)),'units':'rad'}
        else: raise ValueError('Joint read unavailable')
        return {'values':list(_numbers(values,self.dof)),'units':'deg'}

    def move_joint(self,degrees,**options):
        q=list(_numbers(degrees,self.dof)); f=self.family; c=self.client
        if f=='elite_ec':
            if set(options)!={'speed_percent'} or not 0<float(options['speed_percent'])<=100: raise ValueError('Explicit Elite speed_percent 0..100 required')
            external=_numbers(self.config['external_axes_native'],2)
            if not self.config.get('external_axes_evidence'): raise ValueError('Elite external-axis hold values require explicit evidence')
            rc=c.move_joint(q+list(external),speed=float(options['speed_percent']),block=False)
            if rc is not True: raise RuntimeError('Elite move_joint rejected: '+str(rc))
        elif f=='elephant280':
            if set(options)!={'speed_percent'}: raise ValueError('Explicit speed_percent required')
            speed=options['speed_percent']
            if isinstance(speed,bool) or not isinstance(speed,int) or not 1<=speed<=100: raise ValueError('Invalid speed_percent')
            rc=c.send_angles(q,speed)
            if rc==-1: raise RuntimeError('myCobot send_angles failed')
        elif f=='igus_cri':
            if set(options)!={'velocity_percent'}: raise ValueError('Explicit velocity_percent required')
            speed=float(options['velocity_percent'])
            if not math.isfinite(speed) or not 0<speed<=100: raise ValueError('Invalid velocity_percent')
            if not self.config.get('remaining_axes_evidence'): raise ValueError('CRI remaining axes require explicit hold/unused-axis evidence')
            tail=_numbers(self.config['remaining_axes_native'],9-self.dof)
            rc=c.move_joints(*(q+list(tail)),velocity=speed,wait_move_finished=False)
            if rc is not True: raise RuntimeError('CRI motion command rejected')
        elif f=='agilebot_a':
            if set(options)!={'velocity_ratio','acceleration_ratio'}: raise ValueError('Explicit velocity/acceleration ratios required')
            from Agilebot.IR.A.sdk_classes import MotionPose
            from Agilebot.IR.A.sdk_types import PoseType
            from Agilebot.IR.A.status_code import StatusCodeEnum
            v=float(options['velocity_ratio']); a=float(options['acceleration_ratio'])
            if not 0<v<=1 or not 0<a<=1.2: raise ValueError('Invalid motion ratio')
            if not self.config.get('remaining_axes_evidence'): raise ValueError('Agilebot remaining axes require explicit hold/unused-axis evidence')
            tail=_numbers(self.config['remaining_axes_native'],9-self.dof)
            pose=MotionPose(); pose.generate_motion_pose(q+list(tail),PoseType.JOINT)
            rc=c.motion.move_joint(pose,vel=v,acc=a)
            if rc!=StatusCodeEnum.OK: raise RuntimeError(str(rc))
        elif f=='hiwin_commander':
            if options: raise ValueError('HIWIN velocity/trajectory options not qualified')
            values=(ctypes.c_double*6)(*(math.radians(v) for v in q))
            rc=c.unilab_hiwin_move_joints(self.handle,values)
            if rc!=0: raise RuntimeError('HIWIN move: '+str(rc)+' '+c.unilab_hiwin_error().decode(errors='replace'))
        elif f=='flexiv_rdk_1_9':
            if set(options)!={'target_velocity_rad_s','max_velocity_rad_s','max_acceleration_rad_s2'}:
                raise ValueError('Explicit Flexiv velocity and acceleration vectors required')
            vectors=[_numbers(options[k],self.dof) for k in ('target_velocity_rad_s','max_velocity_rad_s','max_acceleration_rad_s2')]
            if any(v<=0 for row in vectors[1:] for v in row): raise ValueError('Invalid Flexiv limits')
            import flexivrdk
            if not c.operational() or c.mode() not in (flexivrdk.Mode.NRT_JOINT_POSITION,flexivrdk.Mode.NRT_JOINT_IMPEDANCE):
                raise RuntimeError('Flexiv deployment must explicitly enable and choose joint mode')
            c.SendJointPosition([math.radians(v) for v in q],*vectors)
        elif f=='estun_eri':
            if options: raise ValueError('ESTUN motion speed/trajectory options are not qualified')
            if not self.config.get('control_acquired_evidence'):
                raise ValueError('Explicit ESTUN controller control-acquisition evidence required')
            units=self.config.get('joint_units')
            if not (units in ('deg','rad') or isinstance(units,list) and len(units)==self.dof and all(u in ('deg','rad','mm','m') for u in units)): raise ValueError('ESTUN units need controller documentation')
            values=q if units=='deg' or isinstance(units,list) else [math.radians(v) for v in q]
            tail=_numbers(self.config['remaining_axes_native'],16-self.dof)
            data=(ctypes.c_double*16)(*(values+list(tail)))
            rc=c.unilab_estun_movj(self.handle,data)
            if rc not in (0,1): raise RuntimeError('ESTUN movJ rejected: '+str(rc))
            return {'sdk_status':rc,'sdk_state':'completed' if rc==1 else 'executing','completed':False}
        else:
            raise ValueError('Motion is not qualified for this native SDK boundary')
        return {'accepted':True,'completed':False}

    def move_cartesian(self,waypoints):
        if self.family!='inovo_commander': raise ValueError('Cartesian entry unavailable')
        from commander_api.motion_control_client import Motion,Waypoint
        if not waypoints: raise ValueError('Explicit Cartesian waypoints required')
        motion=Motion()
        for item in waypoints:
            if set(item)!={'pose_xyzrpy_si','max_joint_velocity','max_joint_acceleration'}:
                raise ValueError('Explicit Cartesian pose and joint motion limits required')
            pose=_numbers(item['pose_xyzrpy_si'],6)
            v=float(item['max_joint_velocity']);a=float(item['max_joint_acceleration'])
            if not math.isfinite(v) or not math.isfinite(a) or v<=0 or a<=0:
                raise ValueError('Invalid motion limits')
            point=Waypoint(*pose).constrain_joint_velocity(v).constrain_joint_acceleration(a)
            motion.add(point)
        self.client.run(motion)
        return {'sdk_action_success':True,'hardware_verified':False}


def main():
    session=None
    for line in sys.stdin:
        request=json.loads(line)
        try:
            if session is None:
                if request['config'].get('loopback_only'):
                    def network_guard(event,args):
                        if event=='socket.connect' and isinstance(args[1],tuple) and args[1][0] not in ('127.0.0.1','::1'):
                            raise PermissionError('Offline validation rejects non-loopback socket connect')
                    # Does not monkeypatch vendor methods or serialization. This
                    # guards Python sockets; native SDKs need OS network isolation.
                    sys.addaudithook(network_guard)
                session=VendorSession(request['family'],request['config'])
            op=request['operation']
            if op not in ('connect','read_joints','move_joint','move_cartesian'): raise ValueError('Unsupported operation')
            result=getattr(session,op)(**request['arguments'])
            reply={'id':request['id'],'ok':True,'result':result}
        except Exception as exc:
            reply={'id':request['id'],'ok':False,'error':type(exc).__name__+': '+str(exc)}
        print('UNILAB_SDK_REPLY '+json.dumps(reply),flush=True)


if __name__=='__main__': main()
