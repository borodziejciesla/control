#ifndef CONTROL_MPC_HPP_
#define CONTROL_MPC_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <unsupported/Eigen/MatrixFunctions>

#include "mpc_calibrations.hpp"

namespace control {

template <size_t state_size, size_t predictions_step_number>
class Mpc {
 public:
  using SetValueArray = std::array<double, predictions_step_number>;
  using ControlArray = std::array<double, predictions_step_number>;
  using StateVector = std::array<double, state_size>;

 public:
  Mpc(void) = default;
  virtual ~Mpc(void) = default;

  void SetCalibrations(const MpcCalibrations<state_size, predictions_step_number>& calibrations) {
    // Set matrices
    ConvertTransitionAndControlMatrix(calibrations);
    SetRAndQ(calibrations);
    SetHAndG();
  }

  void Reset(void) {}

  void SetValue(const SetValueArray& set_value) {
    std::copy(set_value.begin(), set_value.end(), set_value_.begin());
  }

  double GetControl(const StateVector& state) {
    for (auto index = 0u; index < state_size; index++) {
      state_(index) = state.at(index);
    }

    CalculateControl();

    return control_series_.at(0);
  }

 private:
  void ConvertTransitionAndControlMatrix(
      const MpcCalibrations<state_size, predictions_step_number>& calibrations) {
    // Transition
    for (auto row = 0u; row < state_size; row++) {
      for (auto col = 0u; col < state_size; col++) {
        transition_matrix_(row, col) = calibrations.transition_matrix.at(row).at(col);
      }
    }

    // Control
    for (auto row = 0u; row < state_size; row++)
      control_matrix_(row) = calibrations.control_matrix.at(row);
  }

  void SetRAndQ(const MpcCalibrations<state_size, predictions_step_number>& calibrations) {
    // Q
    for (auto step_index = 0u; step_index < predictions_step_number; step_index++) {
      for (auto state_index = 0u; state_index < state_size; state_index++) {
        const auto index = step_index * state_size + state_index;
        q_(index, index) = calibrations.q.at(state_index);
      }
    }

    // R
    for (auto index = 0u; index < predictions_step_number; index++) {
      r_(index, index) = calibrations.r;
    }
  }

  void SetHAndG(void) {
    static Eigen::MatrixPower<Eigen::Matrix<double, state_size, state_size>> tm_pow(
        transition_matrix_);
    // G
    for (auto index = 0u; index < predictions_step_number; index++) {
      g_.block(index * state_size, 0u, state_size, state_size) = tm_pow(index + 1u);
    }

    // H
    for (auto step_index = 0u; step_index < predictions_step_number; step_index++) {
      for (auto index = 0u; index <= step_index; index++) {
        h_.block(index * state_size, step_index, state_size, 1u) = tm_pow(index) * control_matrix_;
      }
    }
  }

  void CalculateControl(void) {
    SetHAndG();

    const auto r = h_.transpose() * q_ * h_ + r_;
    const auto q = h_.transpose() * q_ * g_;

    const auto u = r.inverse() * q * state_;

    for (auto index = 0u; index < predictions_step_number; index++)
      control_series_.at(index) = u(index, 0u);
  }

  SetValueArray set_value_;
  ControlArray control_;
  std::array<double, predictions_step_number> control_series_;

  Eigen::Vector<double, state_size> state_;

  Eigen::Matrix<double, state_size, state_size> transition_matrix_;
  Eigen::Vector<double, state_size> control_matrix_;

  Eigen::Matrix<double, state_size * predictions_step_number, state_size> g_ =
      Eigen::Matrix<double, state_size * predictions_step_number, state_size>::Zero();
  Eigen::Matrix<double, state_size * predictions_step_number, predictions_step_number> h_ =
      Eigen::Matrix<double, state_size * predictions_step_number, predictions_step_number>::Zero();

  Eigen::Matrix<double, predictions_step_number, predictions_step_number> r_ =
      Eigen::Matrix<double, predictions_step_number, predictions_step_number>::Zero();
  Eigen::Matrix<double, state_size * predictions_step_number, state_size * predictions_step_number>
      q_ = Eigen::Matrix<double, state_size * predictions_step_number,
                         state_size * predictions_step_number>::Zero();
};
}  // namespace control

#endif  //  CONTROL_MPC_HPP_
