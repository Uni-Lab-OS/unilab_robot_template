// Thin ABI boundary around an explicitly installed ESTUN ERI SDK.
// Build against the matching vendor headers/libraries; never emulate success.
#include <cstdint>
#include <cmath>
#include <cstring>
#include <new>
#include "ERIParamManager.h"

namespace {
thread_local const char* bridge_error = "";
constexpr int invalid_argument = -10001;
constexpr int sdk_exception = -10002;
int invalid() { bridge_error = "Invalid ESTUN bridge argument"; return invalid_argument; }
int failed() { bridge_error = "Exception from ESTUN SDK"; return sdk_exception; }
}

extern "C" {
const char* unilab_estun_bridge_error() { return bridge_error; }
void* unilab_estun_create() {
    bridge_error = "";
    try { return new ERIParamManager(); }
    catch (...) { failed(); return nullptr; }
}
int unilab_estun_destroy(void* handle) {
    if (!handle) return invalid();
    try { delete static_cast<ERIParamManager*>(handle); return 0; }
    catch (...) { return failed(); }
}
// SDK return 1 means parameters accepted, NOT all channels connected.
int unilab_estun_configure(void* handle, const char* host,
                         unsigned short command_port, unsigned short servo_port,
                         unsigned short status_port) {
    if (!handle || !host || !*host || !command_port || !servo_port || !status_port)
        return invalid();
    try { return static_cast<ERIParamManager*>(handle)->setCommunicationParam(
        host, command_port, servo_port, status_port); }
    catch (...) { return failed(); }
}
int unilab_estun_connection(void* handle, unsigned char* channels) {
    if (!handle || !channels) return invalid();
    try {
        char command = 0, servo = 0, status = 0;
        static_cast<ERIParamManager*>(handle)->getRobotConnStatus(command, servo, status);
        channels[0] = static_cast<unsigned char>(command);
        channels[1] = static_cast<unsigned char>(servo);
        channels[2] = static_cast<unsigned char>(status);
        return 0;
    } catch (...) { return failed(); }
}
// Output always holds exactly 16 SDK-native values; caller applies a pinned mapping.
int unilab_estun_joints(void* handle, double* output16) {
    if (!handle || !output16) return invalid();
    try {
        double values[16] = {};
        bool ok = static_cast<ERIParamManager*>(handle)->getJointValue(values);
        if (ok) std::memcpy(output16, values, sizeof(values));
        return ok ? 1 : 0;
    } catch (...) { return failed(); }
}
// No implicit acquisition of control or enable action. SDK semantics are retained:
// 1 completed, 0 executing, -1 failed. Transport callers must verify fresh feedback.
int unilab_estun_movj(void* handle, const double* values16) {
    if (!handle || !values16) return invalid();
    double values[16];
    for (int i = 0; i < 16; ++i) {
        if (!std::isfinite(values16[i])) return invalid();
        values[i] = values16[i];
    }
    try { return static_cast<ERIParamManager*>(handle)->movJ(values); }
    catch (...) { return failed(); }
}
// The vendor error-string API has no length argument. Do not expose it through
// an unbounded Python buffer; retain the SDK status/error code at this boundary.
int unilab_estun_acquire_control(void* handle) {
    if (!handle) return invalid();
    try { return static_cast<ERIParamManager*>(handle)->getControlRobot() ? 1 : 0; }
    catch (...) { return failed(); }
}
int unilab_estun_release_control(void* handle) {
    if (!handle) return invalid();
    try { return static_cast<ERIParamManager*>(handle)->releaseControlOverRobot() ? 1 : 0; }
    catch (...) { return failed(); }
}
}
