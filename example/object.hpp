#ifndef CONTROL_EXAMPLE_OBJECT_HPP_
#define CONTROL_EXAMPLE_OBJECT_HPP_

#include <Eigen/Dense>

namespace control_example {
  template <size_t state_size>
  class Object {
    public:
      using TransitionMatrix = Eigen::Matrix<double, state_size, state_size>;
      using ControlMatrix = Eigen::Matrix<double, state_size, 1u>;
      using StateVector = Eigen::Vector<double, state_size>;

    public:
      Object(const TransitionMatrix & transition_matrix, const ControlMatrix & control_matrix)
        : transition_matrix_{transition_matrix}
        , control_matrix_{control_matrix} {}

      void SetState(const StateVector & state) {
        state_ = state;
      }

      void Run(const double control) {
        state_ = transition_matrix_ * state_ + control_matrix_ * control;
      }

      const StateVector & GetState(void) {
        return state_;
      }
      
    private:
      TransitionMatrix transition_matrix_;
      ControlMatrix control_matrix_;
      StateVector state_;
  };
}

#endif  //  CONTROL_EXAMPLE_OBJECT_HPP_
