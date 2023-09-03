#ifndef CONTROL_MPC_HPP_
#define CONTROL_MPC_HPP_

#include <algorithm>
#include <array>

#include "mpc_calibrations.hpp"

namespace control {  

  template <size_t state_size, size_t predictions_step_number>
  class Mpc {
    public:
      using SetValueArray = std::array<double, predictions_step_number>;
      using ControlArray = std::array<double, predictions_step_number>;

    public:
      void SetCalibrations(const MpcCalibrations<state_size, predictions_step_number> & calibrations) {
        //
      }

      void Reset(void) {}

      void SetValue(const SetValueArray & set_value) {
        std::copy(set_value.begin(), set_value.end(), set_value_.begin());
      }

      double GetControl(const double y) {
        //TODO: Run control calculation

        return control_.at(0);
      }

    private:
      SetValueArray set_value_;
      ControlArray control_;
      std::array<double, predictions_step_number> control_series_;
  };
}

#endif  //  CONTROL_MPC_HPP_
