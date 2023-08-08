#ifndef CONTROL_INCLUDE_PID_CALIBRATIONS_HPP_
#define CONTROL_INCLUDE_PID_CALIBRATIONS_HPP_

namespace control {
  struct PidCalibrations {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;

    double min_control = 0.0;
    double max_control = 0.0;

    bool use_d_filtering = true;
    bool use_antiwindup = true;
  };
}

#endif  //  CONTROL_INCLUDE_PID_CALIBRATIONS_HPP_