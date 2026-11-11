// C ABI over the official HIWIN Commander client. Acknowledged commands do not
// certify motion completion. Explicitly acquired control is separate from TCP.
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <arpa/inet.h>
#include <hiwin_robot_client_library/commander.hpp>

namespace {
struct Client {
    hrsdk::Commander sdk;
    bool connected = false;
    Client(const char* host, int port) : sdk(host, port) {
        timeval timeout{2,0}; sdk.setReceiveTimeout(timeout);
    }
};
thread_local std::string last_error;
int failed(Client* c, const char* why) {
    last_error = why;
    if(c){c->sdk.close();c->connected=false;}
    return -10002;
}
template<class F> int invoke(void* handle, F action) {
    auto* c=static_cast<Client*>(handle);
    if(!c || !c->connected){last_error="HIWIN transport not connected";return -10001;}
    try { return action(c->sdk); }
    catch(const std::exception& e){return failed(c,e.what());}
    catch(...){return failed(c,"HIWIN SDK exception");}
}
}
extern "C" {
const char* unilab_hiwin_error(){return last_error.c_str();}
void* unilab_hiwin_create(const char* host,int port){
    if(!host || !*host || port<1 || port>65535)return nullptr;
    in_addr address{};if(inet_pton(AF_INET,host,&address)!=1)return nullptr;
    try{return new Client(host,port);}catch(...){return nullptr;}
}
int unilab_hiwin_connect(void* handle){
    auto* c=static_cast<Client*>(handle);
    if(!c || c->connected)return -10001;
    try {c->connected=c->sdk.connect();return c->connected?0:-10003;}
    catch(const std::exception& e){return failed(c,e.what());}
    catch(...){return failed(c,"HIWIN SDK exception");}
}
int unilab_hiwin_destroy(void* handle){
    if(!handle)return -10001;
    delete static_cast<Client*>(handle);return 0;
}
int unilab_hiwin_read_joints(void* handle,double* output6){
    if(!output6)return -10001;
    return invoke(handle,[&](hrsdk::Commander& sdk){
        double values[6];for(auto& x:values)x=std::numeric_limits<double>::quiet_NaN();
        int rc=sdk.getActualPosition(values);
        if(rc!=0)return rc;
        for(auto x:values)if(!std::isfinite(x))throw std::runtime_error("Invalid HIWIN feedback");
        std::memcpy(output6,values,sizeof(values));return 0;
    });
}
int unilab_hiwin_move_joints(void* handle,const double* input6){
    if(!input6)return -10001;
    double values[6];std::ostringstream encoded;
    for(int i=0;i<6;i++){
        if(!std::isfinite(input6[i]))return -10001;
        values[i]=input6[i];if(i)encoded<<',';
        encoded<<std::fixed<<std::setprecision(7)<<(values[i]*180.0/std::acos(-1.0));
    }
    // Upstream stores this text as uint16_t characters in a fixed 247-word area.
    if(encoded.str().size()>247)return -10001;
    return invoke(handle,[&](hrsdk::Commander& sdk){return sdk.ptpJoint(values);});
}
int unilab_hiwin_acquire_control(void* handle){
    return invoke(handle,[](hrsdk::Commander& sdk){return sdk.getPermissions();});
}
int unilab_hiwin_abort(void* handle){
    return invoke(handle,[](hrsdk::Commander& sdk){return sdk.motionAbort();});
}
}
