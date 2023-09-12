#ifndef CONTROL_INCLUDE_MPC_CALIBRATIONS_HPP_
#define CONTROL_INCLUDE_MPC_CALIBRATIONS_HPP_

#include <array>

namespace control {
  template <size_t state_size, size_t predictions_step_number>
  struct MpcCalibrations {
    std::array<double, state_size> q;
    std::array<double, predictions_step_number> r;

    std::array<std::array<double, state_size>, state_size> transition_matrix;
    std::array<double, state_size> control_matrix;
  };
}

#endif  //  CONTROL_INCLUDE_MPC_CALIBRATIONS_HPP_
