#ifndef CONTROL_INCLUDE_MPC_CALIBRATIONS_HPP_
#define CONTROL_INCLUDE_MPC_CALIBRATIONS_HPP_

#include <array>

namespace control {
template <size_t state_size, size_t predictions_step_number>
struct MpcCalibrations {
  double r;
  std::array<double, state_size> q;

  std::array<std::array<double, state_size>, state_size> transition_matrix;
  std::array<double, state_size> control_matrix;
};
}  // namespace control

#endif  //  CONTROL_INCLUDE_MPC_CALIBRATIONS_HPP_
