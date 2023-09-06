#ifndef CONTROL_MPC_HPP_
#define CONTROL_MPC_HPP_

#include <algorithm>
#include <array>

#include <Eigen/Dense>

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

      void SetCalibrations(const MpcCalibrations<state_size, predictions_step_number> & calibrations) {
        // Set matrices
        ConvertTransitionAndControlMatrix(calibrations);
        SetRAndQ(calibrations);
        SetSxAndSu();
      }

      void Reset(void) {}

      void SetValue(const SetValueArray & set_value) {
        std::copy(set_value.begin(), set_value.end(), set_value_.begin());
      }

      double GetControl(const StateVector & state) {
        for (auto index = 0u; index < state_size; index++)
          state_(index, 0u) = state.at(index);

        CalculateControl();

        return control_.at(0);
      }

    private:
      void ConvertTransitionAndControlMatrix(const MpcCalibrations<state_size, predictions_step_number> & calibrations) {
        // Transition
        for (auto row = 0u; row < state_size; row++) {
          for (auto col = 0u; col < state_size; col++) {
            transition_matrix_(row, col) = calibrations.transition_matrix.at(row).at(col);
          }
        }

        // Control
        for (auto row = 0u; row < state_size; row++)
          control_matrix_(row, 0u) = calibrations.control_matrix.at(row);
      }

      void SetRAndQ(const MpcCalibrations<state_size, predictions_step_number> & calibrations) {
        // Q
        q_ = Eigen::Matrix<double, state_size * predictions_step_number, state_size * predictions_step_number>::Zero();
        for (auto step_index = 0u; step_index < predictions_step_number; step_index++) {
          for (auto state_index = 0u; state_index < state_size; state_index++) {
            const auto index = step_index * state_size + state_index;
            q_(index, index) = calibrations.q.at(state_index);
          }
        }

        // R
        r_ = Eigen::Matrix<double, predictions_step_number * predictions_step_number, predictions_step_number * predictions_step_number>::Zero();
        for (auto step_index = 0u; step_index < predictions_step_number; step_index++) {
          for (auto control_index = 0u; control_index < predictions_step_number; control_index++) {
            const auto index = step_index * state_size + control_index;
            r_(index, index) = calibrations.r.at(control_index);
          }
        }
      }

      void SetSxAndSu(void) {
        sx_ = Eigen::Matrix<double, state_size * predictions_step_number, state_size>::Zero();
        su_ = Eigen::Matrix<double, state_size * predictions_step_number, predictions_step_number>::Zero();

        sx_.block(0u, 0u, state_size, state_size) = Eigen::Matrix<double, state_size, state_size>::Identity();

        for (auto step_index = 1u; step_index < predictions_step_number; step_index++) {
          // Sx
          sx_.block(step_index * state_size, 0u, state_size, state_size)
            = sx_.block((step_index - 1u) * state_size, 0u, state_size, state_size) * transition_matrix_;
          // Sy
          for (auto index = 0u; index < predictions_step_number; index++)
            su_.block(step_index * state_size, 1, state_size, 1u) 
              = sx_.block(index * state_size, 0, state_size, state_size) * control_matrix_;
        }
      }

      void CalculateControl(void) {
        const auto h = su_.transpose() * q_ * su_;// + r_;
        const auto f = sx_.transpose() * q_ * sx_;

        const auto u = h.inverse() * f * state_;

        for (auto index = 0u; index < predictions_step_number; index++)
          control_series_.at(index) = u(index, 0u);
      }

      SetValueArray set_value_;
      ControlArray control_;
      std::array<double, predictions_step_number> control_series_;

      Eigen::Matrix<double, state_size, 1u> state_;

      Eigen::Matrix<double, state_size, state_size> transition_matrix_;
      Eigen::Matrix<double, state_size, 1u> control_matrix_;
      Eigen::Matrix<double, state_size * predictions_step_number, state_size * predictions_step_number> q_;
      Eigen::Matrix<double, predictions_step_number * predictions_step_number, predictions_step_number * predictions_step_number> r_;

      Eigen::Matrix<double, state_size * predictions_step_number, state_size> sx_;
      Eigen::Matrix<double, state_size * predictions_step_number, predictions_step_number> su_;
  };
}

#endif  //  CONTROL_MPC_HPP_
