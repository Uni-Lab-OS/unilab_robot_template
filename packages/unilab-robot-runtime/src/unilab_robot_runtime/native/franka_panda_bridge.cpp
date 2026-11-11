// Read-only C boundary for the Panda-compatible libfranka 0.9.2.
// SDK networking remains unmodified. Call from the bounded SDK worker.
#include <franka/robot.h>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
thread_local std::string error;
struct Session { std::unique_ptr<franka::Robot> robot; };
Session& session(void* handle) {
  if (!handle) throw std::invalid_argument("Null Panda session");
  return *static_cast<Session*>(handle);
}
template<class F> int checked(F f) {
  error.clear();
  try { f(); return 0; }
  catch (const std::exception& e) { error=e.what(); return -1; }
  catch (...) { error="Unknown Panda SDK exception"; return -2; }
}
}
extern "C" {
const char* unilab_panda_error() { return error.c_str(); }
void* unilab_panda_create() {
  error.clear();
  try {
    return new Session{nullptr};
  } catch (const std::exception& e) { error=e.what(); return nullptr; }
}
int unilab_panda_destroy(void* handle) { delete static_cast<Session*>(handle); return 0; }
int unilab_panda_connect(void* handle, const char* host) {
  return checked([&] { auto& s=session(handle);
    if (!host || !*host) throw std::invalid_argument("Explicit FCI host required");
    if (s.robot) throw std::logic_error("Already connected");
    s.robot.reset(new franka::Robot(host));
  });
}
int unilab_panda_read_once(void* handle, double* output) {
  return checked([&] { auto& s=session(handle);
    if (!s.robot || !output) throw std::logic_error("Connected session and output required");
    const auto state=s.robot->readOnce();
    for (double q: state.q) if (!std::isfinite(q)) throw std::runtime_error("Non-finite SDK state");
    for (unsigned i=0;i<7;++i) output[i]=state.q[i];
  });
}
int unilab_panda_server_version(void* handle, int* output) {
  return checked([&] { auto& s=session(handle);
    if (!s.robot || !output) throw std::logic_error("Connected session and output required");
    *output=s.robot->serverVersion();
  });
}
}
