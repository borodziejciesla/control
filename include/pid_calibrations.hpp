#ifndef CONTROL_INCLUDE_PID_CALIBRATIONS_HPP_
#define CONTROL_INCLUDE_PID_CALIBRATIONS_HPP_

namespace control {
  enum class PidType {
    PID = 0,
    PI = 1,
    PD = 2,
    P = 3,
    I = 4
  };
  
  struct PidCalibrations {
    double tp = 0.0;

    PidType pid_type = PidType::P;

    double kp = 0.0;
    double ti = 0.0;
    double td = 0.0;

    double nd = 0.0;

    double min_control = 0.0;
    double max_control = 0.0;

    bool use_d_filtering = true;
    bool use_antiwindup = true;
  };
}

#endif  //  CONTROL_INCLUDE_PID_CALIBRATIONS_HPP_
