#include "gtest/gtest.h"

#include "mpc.hpp"

class MpcTests : public ::testing::Test {
    protected:
        void SetUp(void) override {}
};

TEST_F(MpcTests, ConstructorTest) {
    std::unique_ptr<control::Mpc<3u, 10u>> mpc;
    mpc = std::make_unique<control::Mpc<3u, 10u>>();
    //EXPECT_NO_THROW(mpc = std::make_unique<control::Mpc<3u, 10u>>());
}

