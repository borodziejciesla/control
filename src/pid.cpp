#include "pid.hpp"

#include <stdexcept>


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
    ae_ = kp_ * nd_ * td_ / (td_ + nd_ * tp_);
  }

  void Pid::Reset(void) {
    tp_ = 0.0;
    kp_ = 0.0;
    ti_ = 0.0;
    td_ = 0.0;
    value_ = 0.0;
    use_d_filtering_ = true;
    use_antiwindup_ = false;
    min_control_ = 0.0;
    max_control_ = 0.0;
  }

  void Pid::SetValue(const double value) {
    value_ = value;
  }

  double Pid::GetControl(const double y) {
    prev_error_ = error_;
    error_ = y - value_;
    prev_error_derivative_ = error_derivative_;

    control_ = CalculateProportionalPart()
      + CalculateIntegralPart()
      + CalculateDerivativePart();

    saturated_control_ = SaturateControl(control_);

    return saturated_control_;
  }

  bool Pid::MakeSelfTuning(double & control) {
    bool is_tuned = false;
    return true;
  }

  double Pid::CalculateProportionalPart(void) const {
    return error_ * kp_;
  }

  double Pid::CalculateIntegralPart(void) {
    if (use_antiwindup_)
      integrated_signal_ = (error_ * kp_ / ti_) + ((saturated_control_ - control_) / ti_);
    else
      integrated_signal_ = error_ * kp_ / ti_;

    error_integral_ += integrated_signal_ * tp_;

    return error_integral_;
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

  void Pid::SetZieglerNicholsTuning(const PidType pid_type) {
    switch (pid_type) {
      case PidType::P: {
        kp_ = 0.5 * ku_;
        td_ = 0.0;
        ti_ = 0.0;
        return;
      }
      case PidType::PI: {
        kp_ = 0.45 * ku_;
        td_ = 0.83 * tu_;
        ti_ = 0.0;
        return;
      }
      case PidType::PD: {
        kp_ = 0.8 * ku_;
        td_ = 0.0;
        ti_ = 0.125 * tu_;
        return;
      }
      case PidType::PID: {
        kp_ = 0.6 * ku_;
        td_ = 0.125 * tu_;
        ti_ = 0.5 * tu_;
        return;
      }
      default: {
        throw std::invalid_argument("Invalid controller type!");
      }
    }
  }
}