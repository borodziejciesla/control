#ifndef CONTROL_INCLUDE_HPP_
#define CONTROL_INCLUDE_HPP_

#include "pid_calibrations.hpp"

namespace control {
  class Pid {
    public:
      Pid(void) = default;
      virtual ~Pid(void) = default;

      void SetCalibrations(const PidCalibrations & calibrations);
      void Reset(void);

      void SetValue(const double value);
      double GetControl(const double y);

    private:
      double kp_ = 0.0;
      double ki_ = 0.0;
      double kd_ = 0.0;

      double value_ = 0.0;
      double error_ = 0.0;
  };
}

#endif  //  CONTROL_INCLUDE_HPP_