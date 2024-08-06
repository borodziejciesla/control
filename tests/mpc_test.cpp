#include "mpc.hpp"

#include "gtest/gtest.h"

class MpcTests : public ::testing::Test {
 protected:
  void SetUp(void) override {}
};

TEST_F(MpcTests, ConstructorTest) {
  constexpr auto state_size = 3u;
  constexpr auto steps_number = 10u;

  using MpcTest = control::Mpc<state_size, steps_number>;

  std::unique_ptr<MpcTest> mpc;
  EXPECT_NO_THROW(mpc = std::make_unique<MpcTest>());
}

TEST_F(MpcTests, RunCheckExceptionsTest) {
  constexpr auto state_size = 2u;
  constexpr auto steps_number = 2u;

  std::unique_ptr<control::Mpc<state_size, steps_number>> mpc;
  mpc = std::make_unique<control::Mpc<state_size, steps_number>>();

  control::MpcCalibrations<state_size, steps_number> calibrations;
  calibrations.q = {1.0, 1.0};
  calibrations.r = 1.0;
  calibrations.transition_matrix[0][0] = -1.0;
  calibrations.transition_matrix[0][1] = 0.0;
  calibrations.transition_matrix[1][0] = 0.0;
  calibrations.transition_matrix[1][1] = -1.0;
  calibrations.control_matrix = {0.0, 1.0};

  mpc->SetCalibrations(calibrations);
  mpc->SetValue({0.0, 0.0});
  mpc->GetControl({0.0, 0.0});
}