#include "pid.hpp"

#include <cmath>
#include <stdexcept>

namespace control {
void Pid::SetCalibrations(const PidCalibrations& calibrations) {
  MakeSanityCheckOnParameters(calibrations);

  tp_ = calibrations.tp;

  kp_ = calibrations.kp;
  ti_ = calibrations.ti;
  td_ = calibrations.td;

  min_control_ = calibrations.min_control;
  max_control_ = calibrations.max_control;

  use_d_filtering_ = calibrations.use_d_filtering;

  nd_ = calibrations.nd;

  ad_ = -(td_ * nd_ - td_) / (td_);  // td_ / (td_ + nd_ * tp_);
  ae_ = nd_ / td_;                   // -kp_ * nd_ * td_ / (td_ + nd_ * tp_);

  use_antiwindup_ = calibrations.use_antiwindup;
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

void Pid::SetValue(const double value) { value_ = value; }

double Pid::GetControl(const double y) {
  prev_error_ = error_;
  error_ = value_ - y;
  prev_error_derivative_ = error_derivative_;

  control_ = CalculateProportionalPart() + CalculateIntegralPart() + CalculateDerivativePart();

  control_ = SaturateControl(control_);
  control_ = RateLimitControl(control_);

  prev_control_ = control_;

  return control_;
}

double Pid::CalculateProportionalPart(void) const { return error_ * kp_; }

double Pid::CalculateIntegralPart(void) {
  if (use_antiwindup_)
    integrated_signal_ = (error_ * kp_ * ti_) + ((saturated_control_ - control_) * ti_);
  else
    integrated_signal_ = error_ * kp_ * ti_;

  error_integral_ += integrated_signal_ * tp_;
  return error_integral_;
}

double Pid::CalculateDerivativePart(void) {
  if (!use_d_filtering_) {
    return (error_ - prev_error_) / tp_ * td_;
  } else {
    error_derivative_ = ad_ * prev_error_derivative_ + ae_ * (error_ - prev_error_);
    return error_derivative_ * td_ * kp_;
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

double Pid::RateLimitControl(const double control) const {
  const auto control_delta = control - prev_control_;
  const auto control_rate = control_delta / tp_;
  if (std::abs(control_rate) > rate_limit_) {
    return prev_control_ + tp_ * rate_limit_;
  } else {
    return control;
  }
}

void Pid::MakeSanityCheckOnParameters(const PidCalibrations& calibrations) const {
  // General check
  if (calibrations.tp < 0.0)
    throw std::invalid_argument("Pid::MakeSanityCheckOnParameters: Negative tp");

  // For all types
  if (calibrations.kp < 0.0)
    throw std::invalid_argument("Pid::MakeSanityCheckOnParameters: Negative kp");

  // For I, PI, PID
  if (calibrations.ti < 0.0)
    throw std::invalid_argument("Pid::MakeSanityCheckOnParameters: Negative ti");

  // For PD, PID
  if (calibrations.td < 0.0)
    throw std::invalid_argument("Pid::MakeSanityCheckOnParameters: Negative td");

  if (calibrations.use_d_filtering && (calibrations.nd < 0.0))
    throw std::invalid_argument("Pid::MakeSanityCheckOnParameters: Invalid nd");

  // Check saturation
  if (calibrations.max_control < calibrations.min_control)
    throw std::invalid_argument("Pid::MakeSanityCheckOnParameters: Invalid control limitations");
}
}  // namespace control