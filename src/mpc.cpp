#include "mpc.hpp"

namespace control {
  void Mpc::SetCalibrations() {
    //
  }

  void Mpc::Reset() {
    set_value_.clear();
  }

  void Mpc::SetValue(const SetValueVector & setvalue) {
    set_value_ = set_value_;
  }

  double Mpc::GetControl(const double y) {
    return 0.0;
  }
}
