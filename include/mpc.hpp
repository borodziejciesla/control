#ifndef CONTROL_MPC_HPP_
#define CONTROL_MPC_HPP_

#include <vector>

namespace control {
  using SetValueVector = std::vector<double>;

  class Mpc {
    public:
      Mpc(void) = default;
      virtual ~Mpc(void) = default;

      void SetCalibrations();
      void Reset();

      void SetValue(const SetValueVector & setvalue);
      double GetControl(const double y);

    private:
      SetValueVector set_value_;
  };
}

#endif  //  CONTROL_MPC_HPP_
