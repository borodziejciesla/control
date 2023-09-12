#include "gtest/gtest.h"

#include "fslc.hpp"

class FslcTests : public ::testing::Test {
    protected:
        void SetUp(void) override {}
};

TEST_F(FslcTests, ConstructorTest) {
    constexpr auto state_size = 2u;
    constexpr auto steps_number = 2u;

    using FslcTest = control::Fslc<state_size, steps_number>;

    std::unique_ptr<FslcTest> fslc;
    EXPECT_NO_THROW( fslc = std::make_unique<FslcTest>() );
}

TEST_F(FslcTests, RunZeroControlTest) {
    constexpr auto state_size = 2u;
    constexpr auto steps_number = 2u;

    std::unique_ptr<control::Fslc<state_size, steps_number>> fslc;
    fslc = std::make_unique<control::Fslc<state_size, steps_number>>();
    
    control::FslcCalibrations<state_size, steps_number> calibrations;
    calibrations.control_gain[0][0] = 1.0;
    calibrations.control_gain[0][1] = 0.0;
    calibrations.control_gain[1][0] = 0.0;
    calibrations.control_gain[1][1] = 1.0;

    fslc->SetCalibrations(calibrations);
    fslc->SetValue({0.0, 0.0});
    const auto control = fslc->GetControl({0.0, 0.0});

    EXPECT_DOUBLE_EQ(0.0, control.at(0));
    EXPECT_DOUBLE_EQ(0.0, control.at(1));
}

TEST_F(FslcTests, RunNonzeroControlTest) {
    constexpr auto state_size = 2u;
    constexpr auto steps_number = 2u;

    std::unique_ptr<control::Fslc<state_size, steps_number>> fslc;
    fslc = std::make_unique<control::Fslc<state_size, steps_number>>();
    
    control::FslcCalibrations<state_size, steps_number> calibrations;
    calibrations.control_gain[0][0] = 1.0;
    calibrations.control_gain[0][1] = 0.0;
    calibrations.control_gain[1][0] = 0.0;
    calibrations.control_gain[1][1] = 1.0;

    fslc->SetCalibrations(calibrations);
    fslc->SetValue({0.0, 0.0});
    const auto control = fslc->GetControl({2.0, 3.0});

    EXPECT_DOUBLE_EQ(-2.0, control.at(0));
    EXPECT_DOUBLE_EQ(-3.0, control.at(1));
}
