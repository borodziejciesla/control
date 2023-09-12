#include <limits>
#include <vector>

#include "matplotlibcpp.hpp"

#include "fslc.hpp"
#include "mpc.hpp"
#include "pid.hpp"

#include "object.hpp"

namespace plt = matplotlibcpp;

double inf = std::numeric_limits<double>::infinity();

int main(void) {
  /******** Define object ********/
  constexpr auto state_size = 2u;
  constexpr auto control_size = 1u;

  control_example::Object<state_size, control_size>::TransitionMatrix transition_matrix;
  transition_matrix << 0.9048, 0.9048, 0.0, 0.9048;
  control_example::Object<state_size, control_size>::ControlMatrix control_matrix;
  control_matrix << 0.4679, 0.9516;

  control_example::Object<state_size, control_size> object(transition_matrix, control_matrix);
  control_example::Object<state_size, control_size>::ControlVector control_vector;

  control_example::Object<state_size, control_size>::StateVector initial_state;
  initial_state << 1.0, 1.0;
  
  /******** Create controllers ********/
  // Create PID
  control::Pid pid;

  control::PidCalibrations pid_calibrations;
  pid_calibrations.pid_type = control::PidType::PID;
  pid_calibrations.tp = 1.0;
  pid_calibrations.kp = 0.0641982971505295;
  pid_calibrations.ti = 0.0482636982386738;
  pid_calibrations.td = 4.26772888497946;
  pid_calibrations.nd = 1.22748217535378;
  pid_calibrations.min_control = -inf;
  pid_calibrations.max_control = inf;
  pid_calibrations.use_d_filtering = true;
  pid_calibrations.use_antiwindup = false;

  pid.SetCalibrations(pid_calibrations);

  // Create MPC
  constexpr auto prediction_steps_number = 2u;
  control::Mpc<state_size, prediction_steps_number> mpc;
  
  control::MpcCalibrations<state_size, prediction_steps_number> mpc_calibrations;

  mpc_calibrations.q = {1.0, 1.0};
  mpc_calibrations.r = {1.0, 1.0};
  mpc_calibrations.transition_matrix.at(0u) = {0.9048, 0.9048};
  mpc_calibrations.transition_matrix.at(1u) = {0.0, 0.9048};
  mpc_calibrations.control_matrix = {0.4679, 0.9516};
  
  mpc.SetCalibrations(mpc_calibrations);

  // Create FSLC
  control::Fslc<state_size, control_size> fslc;
  
  control::FslcCalibrations<state_size, control_size> fslc_calibrations;

  fslc_calibrations.control_gain[0][0] = 0.3779;
  fslc_calibrations.control_gain[0][1] = 0.9256;
  
  fslc.SetCalibrations(fslc_calibrations);


  /******** Run Simulations ********/
  constexpr auto simulation_length = 100u;

  std::vector<double> timestamps;
  for (auto index = 0u; index < simulation_length; index++)
    timestamps.push_back(static_cast<double>(index));

  // No control
  object.SetState(initial_state);

  std::vector<double> uncontrolled_x1;
  std::vector<double> uncontrolled_x2;
  std::vector<double> uncontrolled_output;

  for (auto index = 0u; index < simulation_length; index++) {
    const auto state = object.GetState();
    uncontrolled_x1.push_back(state(0u));
    uncontrolled_x2.push_back(state(1u));

    const auto y = state(0u);
    uncontrolled_output.push_back(y);

    control_vector << 0.0;

    object.Run(control_vector);
  }

  // PID
  object.SetState(initial_state);

  std::vector<double> pid_control;
  std::vector<double> pid_x1;
  std::vector<double> pid_x2;
  std::vector<double> pid_output;

  pid.SetValue(0.0);

  for (auto index = 0u; index < simulation_length; index++) {
    const auto state = object.GetState();
    pid_x1.push_back(state(0u));
    pid_x2.push_back(state(1u));

    const auto y = state(0u);
    pid_output.push_back(y);

    const auto control = pid.GetControl(y);
    pid_control.push_back(control);

    control_vector << control;

    object.Run(control_vector);
  }

  // MPC
  object.SetState(initial_state);

  std::vector<double> mpc_control;
  std::vector<double> mpc_x1;
  std::vector<double> mpc_x2;
  std::vector<double> mpc_output;

  control::Mpc<state_size, prediction_steps_number>::SetValueArray mpc_value = {};
  mpc.SetValue(mpc_value);

  for (auto index = 0u; index < simulation_length; index++) {
    const auto state = object.GetState();
    mpc_x1.push_back(state(0u));
    mpc_x2.push_back(state(1u));

    const auto y = state(0u);
    mpc_output.push_back(y);

    std::array<double, 2u> state_array = {state(0u), state(1u)}; 
    const auto control = mpc.GetControl(state_array);
    mpc_control.push_back(control);

    control_vector << control;

    object.Run(control_vector);
  }

  // FSLC
  object.SetState(initial_state);

  std::vector<double> fslc_control;
  std::vector<double> fslc_x1;
  std::vector<double> fslc_x2;
  std::vector<double> fslc_output;

  control::Fslc<state_size, control_size>::StateVectorInput fslc_value = {};
  fslc.SetValue(fslc_value);

  for (auto index = 0u; index < simulation_length; index++) {
    const auto state = object.GetState();
    fslc_x1.push_back(state(0u));
    fslc_x2.push_back(state(1u));

    const auto y = state(0u);
    fslc_output.push_back(y);

    std::array<double, 2u> state_array = {state(0u), state(1u)}; 
    const auto control = fslc.GetControl(state_array);
    fslc_control.push_back(control.at(0));

    control_vector << control.at(0);

    object.Run(control_vector);
  }


  /******** Make Plots ********/
  // Output
  plt::figure_size(500, 500);
  plt::named_plot("No Control", timestamps, uncontrolled_output);
  plt::named_plot("PID", timestamps, pid_output);
  plt::named_plot("MPC", timestamps, mpc_output);
  plt::named_plot("FSLC", timestamps, fslc_output);
  plt::title("Output");
  plt::legend();
  plt::save("./output.png");

  // Control
  plt::figure_size(500, 500);
  plt::named_plot("PID", timestamps, pid_control);
  plt::named_plot("MPC", timestamps, mpc_control);
  plt::named_plot("FSLC", timestamps, fslc_control);
  plt::title("Control");
  plt::legend();
  plt::save("./control.png");

  return EXIT_SUCCESS;
}
