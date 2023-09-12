#ifndef CONTROL_INCLUDE_FSLC_HPP_
#define CONTROL_INCLUDE_FSLC_HPP_

#include <array>

#include <Eigen/Dense>

#include "fslc_calibrations.hpp"

namespace control {
  // Full State Linear Controller
  template <size_t state_size, size_t control_size>
  class Fslc {
    public:
      using StateVectorInput = std::array<double, state_size>;
      using ControlVectorOutput = std::array<double, control_size>;

      using StateVector = Eigen::Vector<double, state_size>;
      using ControlVector = Eigen::Vector<double, control_size>;

      using ControlGainMatrix = Eigen::Matrix<double, state_size, control_size>;

    public:
      Fslc(void) = default;
      virtual ~Fslc(void) = default;

      void SetCalibrations(const FslcCalibrations<state_size, control_size> & calibrations) {
        for (auto row = 0u; row < state_size; row++) {
          for (auto col = 0u; col < control_size; col++)
            control_gain_(row, col) = calibrations.control_gain.at(row).at(col);
        }
      }

      void SetValue(const StateVectorInput & set_value) {
        std::copy(set_value.begin(), set_value.end(), set_value_.begin());
      }

      ControlVectorOutput GetControl(const StateVectorInput & state) {
        // Set state value
        std::copy(state.begin(), state.end(), state_.begin());

        // Find error
        const auto error = set_value_ - state_;

        // Find control
        ControlVector control = control_gain_ * error;

        // Set output control
        std::copy(control.begin(), control.end(), control_.begin());

        return control_;
      }

    protected:
      ControlGainMatrix control_gain_;
      StateVector set_value_;
      StateVector state_;
      ControlVectorOutput control_;
  };
} // namespace control

#endif  //  CONTROL_INCLUDE_FSLC_HPP_
