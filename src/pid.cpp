#include "pid.hpp"

namespace control {
  void Pid::SetCalibrations(const PidCalibrations & calibrations) {
    kp_ = calibrations.kp;
    ki_ = calibrations.ki;
    kd_ = calibrations.kd;
  }

  void Pid::Reset(void) {
    kp_ = 0.0;
    ki_ = 0.0;
    kd_ = 0.0;
    value_ = 0.0;
  }

  void Pid::SetValue(const double value) {
    value_ = value;
  }

  double Pid::GetControl(const double y) {
    error_ = y - value_;
  }
}