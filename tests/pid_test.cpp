#include "gtest/gtest.h"

#include "pid.hpp"

class PidTests : public ::testing::Test {
  protected:
    void SetUp(void) override {}
};

TEST_F(PidTests, ConstructorTest) {
  std::unique_ptr<control::Pid> pid;
  EXPECT_NO_THROW(pid = std::make_unique<control::Pid>());
}

