#ifndef CONTROL_INCLUDE_PID_TYPE_HPP_
#define CONTROL_INCLUDE_PID_TYPE_HPP_

namespace control {
  enum class PidType {
    P = 0u,
    PI = 1u,
    PD = 2u,
    PID = 3u
  };
}

#endif  //  CONTROL_INCLUDE_PID_TYPE_HPP_
