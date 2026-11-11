"""Explicit external RoboDK driver transport; no implicit controller or motion.

The process speaks the documented RoboDK stdin/stdout protocol, not the RoboDK
GUI API. A protocol receipt is not hardware commissioning. The caller supplies
the calibrated controller coordinate mapping and deployment evidence.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import queue
import re
import subprocess
import threading
import time
from typing import Callable, Mapping, Sequence


class DriverProtocolError(RuntimeError):
    """Driver failed or returned an invalid/ambiguous response."""


class DriverTimeout(DriverProtocolError):
    """A timed-out session cannot be reused or silently retried."""


def _numbers(values: Sequence[float], count: int) -> tuple[float, ...]:
    result = tuple(float(x) for x in values)
    if len(result) != count or not all(math.isfinite(x) for x in result):
        raise ValueError(f"Expected {count} finite coordinates")
    return result


@dataclass(frozen=True)
class ControllerJointMap:
    """Controller SI = matrix * model SI coordinates + offset_rad.

    Non-diagonal mappings support coupled robot coordinates. No model-name
    inferred identity mapping is supplied by the factory. The legacy offset_rad
    name is retained; mixed-axis offsets use radians/metres per explicit type.
    Native transports can declare per-axis units with source evidence.
    """
    matrix: tuple[tuple[float, ...], ...]
    offset_rad: tuple[float, ...]
    lower_si: tuple[float, ...]
    upper_si: tuple[float, ...]
    evidence: str
    axis_types: tuple[str, ...] = ()
    native_units: tuple[str, ...] = ()
    unit_evidence: str = ''

    def __post_init__(self):
        n = len(self.offset_rad)
        types=self.axis_types or ('revolute',)*n
        units=self.native_units or ('deg',)*n
        if len(types)!=n or len(units)!=n or any((t,u) not in (('revolute','deg'),('revolute','rad'),('prismatic','mm'),('prismatic','m')) for t,u in zip(types,units)):
            raise ValueError('Explicit dimension-compatible axis types and controller units required')
        if ('prismatic' in types or any(u!='deg' for u in units)) and not self.unit_evidence.strip():
            raise ValueError('Non-default controller units require source evidence')
        if not 1 <= n <= 7 or not self.evidence.strip() or len(self.matrix) != n:
            raise ValueError("Explicit 1..7 rotary-axis mapping evidence required")
        for row in self.matrix:
            _numbers(row, n)
        for row in (self.offset_rad, self.lower_si, self.upper_si):
            _numbers(row, n)
        if any(a >= b for a, b in zip(self.lower_si, self.upper_si)):
            raise ValueError("Invalid model joint limits")
        self.from_native(self.to_native(tuple((a+b)/2 for a,b in zip(self.lower_si,self.upper_si))))

    @property
    def dof(self):
        return len(self.offset_rad)

    def to_degrees(self, q):
        if self.axis_types and any(t!='revolute' for t in self.axis_types): raise ValueError('Degree-only transport cannot encode prismatic axes')
        if self.native_units and any(u!='deg' for u in self.native_units): raise ValueError('Degree-only transport requires degree mapping')
        return self.to_native(q)

    def to_native(self, q):
        q = _numbers(q, self.dof)
        if any(x < a or x > b for x,a,b in zip(q,self.lower_si,self.upper_si)):
            raise ValueError("Model joint limit violation")
        factors={'deg':180/math.pi,'rad':1.,'mm':1000.,'m':1.}
        return tuple((sum(a*b for a,b in zip(row,q))+off)*factors[u]
                     for row,off,u in zip(self.matrix,self.offset_rad,self.native_units or ('deg',)*self.dof))

    def from_degrees(self, q):
        if self.axis_types and any(t!='revolute' for t in self.axis_types): raise ValueError('Degree-only transport cannot decode prismatic axes')
        if self.native_units and any(u!='deg' for u in self.native_units): raise ValueError('Degree-only transport requires degree mapping')
        return self.from_native(q)

    def from_native(self, q):
        q = _numbers(q, self.dof)
        # Small dense systems, without adding a numerical dependency to drivers.
        factors={'deg':180/math.pi,'rad':1.,'mm':1000.,'m':1.}
        a = [list(row)+[x/factors[u]-off] for row,x,off,u in zip(self.matrix,q,self.offset_rad,self.native_units or ('deg',)*self.dof)]
        for i in range(self.dof):
            pivot = max(range(i,self.dof), key=lambda j: abs(a[j][i]))
            if abs(a[pivot][i]) < 1e-12:
                raise ValueError("Singular controller joint mapping")
            a[i],a[pivot] = a[pivot],a[i]
            scale = a[i][i]; a[i] = [v/scale for v in a[i]]
            for j in range(self.dof):
                if j != i:
                    scale = a[j][i]; a[j] = [v-scale*w for v,w in zip(a[j],a[i])]
        return tuple(row[-1] for row in a)


class LineProcess:
    """Bounded, serialized line exchanges with an explicitly supplied process.

    No shell is used. stderr is merged into the diagnostic transcript; no
    credentials belong in command arguments. Killing a driver is NOT a robot
    stop and never generates a completed motion receipt.
    """
    def __init__(self, argv: Sequence[str], *, cwd: str | Path, timeout_s: float = 10, env=None):
        if not argv or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("Executable and finite positive timeout required")
        self.argv = tuple(argv); self.cwd = str(cwd); self.timeout_s = timeout_s
        self.process = None; self.lines = queue.Queue(); self.lock = threading.Lock()
        self.env=env
        self.transcript: list[dict] = []; self.failed = False

    def start(self):
        if self.process is not None:
            raise DriverProtocolError("Session already started")
        self.process = subprocess.Popen(self.argv, cwd=self.cwd, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8",
            errors="replace", bufsize=1, env=self.env,creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0))
        def reader():
            for line in self.process.stdout:
                self.lines.put(line.rstrip("\r\n"))
            self.lines.put(None)
        self.reader = threading.Thread(target=reader, daemon=True); self.reader.start()

    def exchange(self, command: str, *, require_joints=False, disconnect=False):
        if "\n" in command or "\r" in command or not command:
            raise ValueError("Exactly one driver command required")
        if self.failed or self.process is None or self.process.poll() is not None:
            raise DriverProtocolError("Driver session unavailable; explicit reconnection required")
        if not self.lock.acquire(blocking=False):
            raise DriverProtocolError("Concurrent commands are not supported")
        try:
            # A previous response/startup line already queued cannot acknowledge
            # a command that has not been written yet.
            while not self.lines.empty():
                stale=self.lines.get_nowait()
                self.transcript.append({'direction':'discard_before_send','line':stale})
            self.transcript.append({"direction":"send","line":command})
            self.process.stdin.write(command+"\n"); self.process.stdin.flush()
            deadline = time.monotonic()+self.timeout_s; joints = None; responses=[]; working=False
            while True:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise DriverTimeout("Driver deadline exceeded; command outcome unknown")
                try:
                    line = self.lines.get(timeout=remaining)
                except queue.Empty as exc:
                    raise DriverTimeout("Driver deadline exceeded; command outcome unknown") from exc
                if line is None:
                    raise DriverProtocolError("Driver exited before terminal response")
                responses.append(line); self.transcript.append({"direction":"receive","line":line})
                if len(responses)>10000:
                    raise DriverProtocolError("Driver response limit exceeded")
                if working and (line.startswith("JNTS ") or line.startswith("JNTS:")):
                    try:
                        joints=tuple(float(x) for x in re.split(r"[,\s]+",line[5:].strip()))
                    except ValueError as exc:
                        raise DriverProtocolError("Malformed JNTS") from exc
                if line.startswith("SMS:Working"):
                    working=True
                if line == "SMS:Ready":
                    if require_joints and joints is None:
                        # A startup Ready must not satisfy a requested joint read.
                        continue
                    if command.split()[0] in ('CONNECT','MOVJ') and not working:
                        # No request IDs exist in this protocol. Require a fresh
                        # Working->Ready transition; startup/stale Ready is not
                        # sufficient acknowledgment of a new motion request.
                        continue
                    return {"lines":responses,"joints_degrees":joints}
                if disconnect and line == "SMS:Disconnected":
                    return {"lines":responses,"joints_degrees":joints}
                if line.startswith("SMS:"):
                    status=line[4:].lower()
                    if command.startswith('CONNECT ') and not working and status in ('disconnected','not connected'):
                        continue  # A common driver startup status.
                    if any(x in status for x in ("error", "failed", "problem", "not connected", "disconnected", "unknown", "invalid", "not supported")):
                        raise DriverProtocolError(line)
        except (OSError, DriverProtocolError):
            self.failed = True
            self.close()
            raise
        finally:
            self.lock.release()

    def close(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait(timeout=2)
        for stream in (self.process.stdin,self.process.stdout):
            if stream:
                stream.close()


class RoboDKDriverAdapter:
    """Explicit controller-coordinate transport and existing SDK-port bridge.

    Pose arguments must already use the deployed driver's XYZRPW convention;
    this layer does not infer TCP/base or convert an arbitrary Euler convention.
    Targets are deployment allowlisted and return only local-session receipts.
    """
    def __init__(self, process: LineProcess, mapping: ControllerJointMap, *,
                 controller: str, deployment_evidence: str, offline: bool,
                 targets: Mapping[str, tuple[Sequence[float], Sequence[float]]] | None = None,
                 completion_tolerance_rad: float = .002, feedback_indices=None, feedback_count=None):
        if not controller.strip() or not deployment_evidence.strip():
            raise ValueError("Controller selection and deployment evidence required")
        if not 0 < completion_tolerance_rad <= .01:
            raise ValueError("Invalid completion tolerance")
        self.process=process; self.mapping=mapping; self.controller=controller
        self.deployment_evidence=deployment_evidence; self.offline=offline
        self.targets=dict(targets or {}); self.tolerance=completion_tolerance_rad
        self.connected=False; self.receipts={}; self.sent_ids=set()
        self.feedback_indices=tuple(feedback_indices) if feedback_indices is not None else None
        self.feedback_count=feedback_count
        if self.feedback_indices is not None and (feedback_count is None or len(self.feedback_indices)!=mapping.dof or len(set(self.feedback_indices))!=mapping.dof or any(not isinstance(i,int) or i<0 or i>=feedback_count for i in self.feedback_indices)):
            raise ValueError('Explicit feedback layout must cover each model joint exactly once')

    def connect(self, host: str, port: int):
        if not re.fullmatch(r"[A-Za-z0-9_.:-]+",host) or not 1 <= port <= 65535:
            raise ValueError("Explicit host and port required")
        self.process.start()
        try:
            self.process.exchange(f"CONNECT {host} {port} {self.mapping.dof}")
            self.connected=True
            return self.read_joints()  # Verify fresh joint feedback, not Ready alone.
        except Exception:
            self.connected=False; self.process.close(); raise

    def read_joints(self):
        if not self.connected:
            raise DriverProtocolError("Not connected")
        try:
            r=self.process.exchange("CJNT",require_joints=True)
        except DriverProtocolError:
            self.connected=False
            raise
        try:
            values=r['joints_degrees']
            if self.feedback_indices is not None:
                values=_numbers(values,self.feedback_count)
                values=tuple(values[i] for i in self.feedback_indices)
            return self.mapping.from_degrees(values)
        except ValueError as exc:
            self.connected=False; self.process.failed=True; self.process.close()
            raise DriverProtocolError("Invalid controller joint feedback") from exc

    def move_joint(self, q_si, pose_xyzrpw, *, command_id: str):
        if not self.connected:
            raise DriverProtocolError("Not connected")
        if not command_id or command_id in self.sent_ids:
            raise ValueError("Nonempty unique command_id required; no automatic replay")
        q=_numbers(q_si,self.mapping.dof); degrees=self.mapping.to_degrees(q)
        pose=_numbers(pose_xyzrpw,6)
        self.sent_ids.add(command_id)
        try:
            self.process.exchange("MOVJ "+" ".join(format(v,".12g") for v in (*degrees,*pose)))
            observed=self.read_joints()
            if max(abs(a-b) for a,b in zip(q,observed)) > self.tolerance:
                raise DriverProtocolError("Driver Ready did not confirm target joint position")
        except Exception:
            self.connected=False; self.process.failed=True; self.process.close(); raise
        receipt={"command_id":command_id,"state":"succeeded","completed":True,
            "witness":"driver_ready_and_fresh_joint_feedback","scope":"offline_protocol_fixture" if self.offline else "controller_driver_report",
            "hardware_verified":False,"joint_positions_si":list(observed)}
        self.receipts[command_id]=receipt
        return receipt

    def execute_target(self, target_ref, parameters, command_id):
        if target_ref not in self.targets:
            raise ValueError("Target is not deployment-allowlisted")
        unsupported=set(parameters)-{'resolved_target'}
        if unsupported:
            # Speed/acceleration/blending units vary by controller (Dobot uses
            # percentages). Never silently discard a caller's motion semantics.
            raise ValueError('Unsupported motion parameters: '+str(sorted(unsupported)))
        q,pose=self.targets[target_ref]
        resolved=parameters.get("resolved_target",{})
        if resolved.get("type") != "joint_positions" or tuple(resolved.get("joint_positions_si",())) != tuple(q):
            raise ValueError("Resolved target does not match deployment target")
        # The existing SDK port calls once per segment with one command ID.
        segment_id=command_id+":"+target_ref
        receipt=self.move_joint(q,pose,command_id=segment_id)
        return {**receipt,"command_id":command_id,"segment_id":segment_id}

    def query_command(self, command_id):
        # Process-local segment receipts cannot prove an entire multi-segment
        # OS command, particularly after restart. Reconciliation stays unknown.
        return self.receipts.get(command_id)

    def request_stop(self, command_id, reason):
        # STOP support/completion is controller-specific. A generic Ready is
        # insufficient proof of a stopped robot; never fabricate confirmation.
        return False

    def close(self):
        try:
            if self.connected and not self.process.failed:
                self.process.exchange("DISCONNECT",disconnect=True)
        finally:
            self.connected=False; self.process.close()


def installed_driver_command(candidate: dict, installation: str | Path, *, python_executable: str | None = None, runtime_directory=None):
    """Resolve a recorded driver only in an explicit external installation."""
    root=Path(installation).resolve(); path=(root/candidate["relative_path"]).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError("Driver is absent from the explicit installation")
    if hashlib.sha256(path.read_bytes()).hexdigest()!=candidate["sha256"]:
        raise ValueError("Driver version differs from recorded source; re-inventory required")
    if path.suffix.lower()==".py":
        if python_executable is None:
            raise ValueError("Explicit compatible driver Python interpreter required")
        compiled=candidate.get('compiled_entrypoints',[])
        if compiled:
            result=subprocess.run([python_executable,'-I','-B','-c',
                'import sys;print("v"+str(sys.version_info.major)+str(sys.version_info.minor))'],
                capture_output=True,text=True,timeout=10,check=True,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            version=result.stdout.strip()
            entries=[e for e in compiled if Path(e['relative_path']).parent.name==version]
            if len(entries)!=1:
                raise ValueError('Driver has no recorded compiled entrypoint for interpreter '+version)
            entry=(root/entries[0]['relative_path']).resolve()
            if not entry.is_relative_to(root) or not entry.is_file() or hashlib.sha256(entry.read_bytes()).hexdigest()!=entries[0]['sha256']:
                raise ValueError('Compiled driver entrypoint differs from recorded source')
        return [python_executable,"-B","-u",str(path)],path.parent
    if path.suffix.lower()!=".exe":
        raise ValueError("Unsupported external driver executable")
    if runtime_directory is not None:
        # Some RoboDK installs put x86 Qt beside x64 drivers. An explicit external
        # launch directory avoids loading that incompatible adjacent DLL. The
        # original executable is hard-linked, never shipped in our wheel.
        runtime=Path(runtime_directory).resolve();runtime.mkdir(parents=True,exist_ok=True)
        target=runtime/path.name
        if not target.exists():os.link(path,target)
        if hashlib.sha256(target.read_bytes()).hexdigest()!=candidate['sha256']:
            raise ValueError('External launch executable hash mismatch')
        return [str(target)],runtime
    return [str(path)],path.parent


def create_driver(catalog, model: str, driver_id: str, *, mapping: ControllerJointMap,
                  controller: str, deployment_evidence: str, robodk_root=None,
                  python_executable=None, offline_process: LineProcess | None = None,
                  targets=None, timeout_s=10, native_config=None, offline=False,
                  runtime_directory=None,feedback_indices=None,feedback_count=None):
    """No network or process starts until explicit ``connect``.

    Selecting a candidate acknowledges controller compatibility remains a
    deployment responsibility. SDK workers require explicit installed runtimes.
    """
    definition=catalog.module_definition(model)
    candidates=definition.get("drivers",{}).get("hardware_candidates",[])
    found=[c for c in candidates if c["id"]==driver_id]
    if len(found)!=1:
        raise ValueError("No callable adapter for selected exact driver candidate")
    candidate=found[0]
    parameters=definition.get('motion_model',{}).get('parameters',{})
    declared=parameters.get('product_catalog',{}).get('axes',{}).get('value')
    if declared is not None and int(declared)!=mapping.dof:
        raise ValueError('Joint mapping DOF differs from selected model')
    joint_types=parameters.get('robot_parameters',{}).get('joint_types',[])
    if candidate.get('adapter')=='vendor_native':
        from .native_sdk import NativeSDKAdapter
        if not python_executable or native_config is None:
            raise ValueError('Explicit installed SDK interpreter and pinned configuration required')
        if controller not in candidate.get('controller_choices',[]):
            raise ValueError('Explicit compatible controller selection required')
        if native_config.get('dof')!=mapping.dof:
            raise ValueError('Native SDK DOF differs from calibrated mapping')
        explicit_types=native_config.get('joint_types')
        if explicit_types!=list(mapping.axis_types or ('revolute',)*mapping.dof) or not native_config.get('joint_type_evidence'):
            raise ValueError('Explicit axis declaration and mapping evidence required; missing source types are not inferred')
        if 'prismatic' in explicit_types:
            if candidate['native_family'] not in ('igus_cri','estun_eri') or native_config.get('joint_units')!=list(mapping.native_units) or not mapping.unit_evidence:
                raise ValueError('Mixed axes require a qualified per-axis SDK unit mapping')
        if mapping.native_units and any(u!='deg' for u in mapping.native_units):
            if candidate['native_family'] not in ('igus_cri','estun_eri') or native_config.get('joint_units')!=list(mapping.native_units):
                raise ValueError('This SDK boundary requires native degree mapping')
        if joint_types and list(joint_types)!=explicit_types:
            raise ValueError('Deployment axis types conflict with source model')
        return NativeSDKAdapter(candidate['native_family'],mapping,
            python_executable=python_executable,deployment_evidence=deployment_evidence,
            config=native_config,timeout_s=timeout_s,targets=targets,offline=offline)
    if candidate.get('adapter')!='robodk_stdio':
        raise ValueError('No callable adapter for selected exact driver candidate')
    if 'prismatic' in joint_types or 'prismatic' in mapping.axis_types:
        raise ValueError('RoboDK degree-only adapter cannot drive prismatic joints')
    if controller not in candidate["controller_choices"]:
        raise ValueError("Explicit compatible controller family selection required")
    if offline_process is None:
        if robodk_root is None:
            raise ValueError("Explicit RoboDK installation required")
        argv,cwd=installed_driver_command(candidate,robodk_root,python_executable=python_executable,runtime_directory=runtime_directory)
        env=dict(os.environ,PATH=str(Path(robodk_root)/'bin')+os.pathsep+os.environ.get('PATH',''))
        process=LineProcess(argv,cwd=cwd,timeout_s=timeout_s,env=env)
    else:
        process=offline_process
    if feedback_indices is None and candidate.get('feedback_layout'):
        feedback_indices=candidate['feedback_layout']['arm_indices']
        feedback_count=candidate['feedback_layout']['count']
    return RoboDKDriverAdapter(process,mapping,controller=controller,
        deployment_evidence=deployment_evidence,offline=offline_process is not None or offline,targets=targets,
        feedback_indices=feedback_indices,feedback_count=feedback_count)
