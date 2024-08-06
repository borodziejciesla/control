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
      double CalculateProportionalPart(void) const;
      double CalculateIntegralPart(void);
      double CalculateDerivativePart(void);

      double SaturateControl(const double control) const;

      void MakeSanityCheckOnParameters(const PidCalibrations & calibrations) const;

      double tp_ = 0.0;

      double kp_ = 0.0;
      double ti_ = 0.0;
      double td_ = 0.0;

      double min_control_ = 0.0;
      double max_control_ = 0.0;

      double value_ = 0.0;
      double error_ = 0.0;
      double prev_error_ = 0.0;
      double error_integral_ = 0.0;
      double error_derivative_ = 0.0;
      double prev_error_derivative_ = 0.0;
      double integrated_signal_ = 0.0;

      bool use_d_filtering_ = true;
      bool use_antiwindup_ = true;

      double ad_ = 0.0;
      double ae_ = 0.0;
      double nd_ = 0.0;

      double control_ = 0.0;
      double saturated_control_ = 0.0;
  };
}

#endif  //  CONTROL_INCLUDE_HPP_