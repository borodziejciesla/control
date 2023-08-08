#include "pid.hpp"

namespace control {
  void Pid::SetCalibrations(const PidCalibrations & calibrations) {
    tp_ = calibrations.tp;

    kp_ = calibrations.kp;
    ti_ = calibrations.ti;
    td_ = calibrations.td;

    min_control_ = calibrations.min_control;
    max_control_ = calibrations.max_control;

    use_d_filtering_ = calibrations.use_d_filtering;

    nd_ = calibrations.nd;

    ad_ = td_ / (td_ + nd_ * tp_);
    ae_ = k * nd_ * td_ / (td_ + nd_ * tp_);
  }

  void Pid::Reset(void) {
    kp_ = 0.0;
    ti_ = 0.0;
    td_ = 0.0;
    value_ = 0.0;
    use_d_filtering_ = true;
  }

  void Pid::SetValue(const double value) {
    value_ = value;
  }

  double Pid::GetControl(const double y) {
    prev_error_ = error_;
    error_ = y - value_;
    prev_error_derivative_ = error_derivative_;

    const auto control = CalculateProportionalPart()
      + CalculateIntegralPart()
      + CalculateDerivativePart();

    return SaturateControl(control);
  }

  double Pid::CalculateProportionalPart(void) const {
    return error_ * kp_;
  }

  double Pid::CalculateIntegralPart(void) {
    error_integral_ += error_ * tp_;
    return error_integral_ * kp_ * tp_ / ti_;
  }

  double Pid::CalculateDerivativePart(void) {
    if (use_d_filtering_) {
      return (error_ - prev_error_) * tp_ * td_;
    } else {
      error_derivative_ = ad_ * prev_error_derivative_ - ae_ * (error_ - prev_error_);
      return error_derivative_;
    }
  }

  double Pid::SaturateControl(const double control) const {
    if (control < min_control_)
      return min_control_;
    else if (control > max_control_)
      return max_control_;
    else
      return control;
  }
}