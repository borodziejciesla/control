#ifndef CONTROL_INCLUDE_FSLC_CALIBRATIONS_HPP_
#define CONTROL_INCLUDE_FSLC_CALIBRATIONS_HPP_

#include <array>

namespace control {
  template <size_t state_size, size_t control_size>
  struct FslcCalibrations {
    std::array<std::array<double, control_size>, state_size> control_gain;
  };
} //  namespace control

#endif  //  CONTROL_INCLUDE_FSLC_CALIBRATIONS_HPP_
