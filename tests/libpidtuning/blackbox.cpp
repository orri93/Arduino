#include <cmath>

#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <gos/arduino/tools/pid/tuning/blackbox.h>

namespace gat = ::gos::arduino::tools;
namespace gatp = ::gos::arduino::tools::pid;
namespace gatpt = ::gos::arduino::tools::pid::tuning;

typedef std::vector<gat::pid::types::Real> DoubleVector;

class BlackBoxFixture : public ::testing::Test {
public:
  void SetUp() {
  }

  void TearDown() {
  }
};

TEST_F(BlackBoxFixture, Initialize) {
  gatpt::black::box::Parameters parameters;
  gatpt::black::box::Variables variables;

  parameters.Sd = 1.0;
  parameters.Kp = gat::types::make_range<gat::pid::types::Real>(1.0, 5.0);
  parameters.Ki = gat::types::make_range<gat::pid::types::Real>(0.001, 0.01);

  gatpt::black::box::initialize(parameters, variables);

  EXPECT_FLOAT_EQ(3.0F, variables.Kp);
  EXPECT_FLOAT_EQ(0.0055F, variables.Ki);

  gatpt::black::box::initialize(parameters, variables, 2.0, 0.005);
  EXPECT_FLOAT_EQ(2.0F, variables.Kp);
  EXPECT_FLOAT_EQ(0.005F, variables.Ki);
}

TEST_F(BlackBoxFixture, RandomNext) {
  gat::pid::types::Real mean, diff, sum, r, sd;

  gatpt::black::box::Parameters parameters;
  gatpt::black::box::Variables variables;

  parameters.Sd = 1.0;
  parameters.Kp = gat::types::make_range<gat::pid::types::Real>(1.0, 5.0);
  parameters.Ki = gat::types::make_range<gat::pid::types::Real>(0.001, 0.01);

  gatpt::black::box::initialize(parameters, variables);

  gatpt::black::box::seed(0.0);

  const int Count = 20000;

  DoubleVector dvector;

  sum = 0.0;
  gat::pid::types::Real minimum =
    std::numeric_limits<gat::pid::types::Real>::lowest();
  gat::pid::types::Real maximum =
    std::numeric_limits<gat::pid::types::Real>::min();
  for (int i = 0; i < Count; i++) {
    r = gatpt::black::box::random::next();
    if (r > maximum) {
      maximum = r;
    }
    if (r < minimum) {
      minimum = r;
    }
    dvector.push_back(r);
    sum += r;
  }
  mean = sum / static_cast<gat::pid::types::Real>(Count);
  sum = 0.0;
  for (auto v : dvector) {
    diff = mean - v;
    sum += diff * diff;
  }
  sd = sqrt(sum / Count);

  EXPECT_TRUE(abs(mean) < 0.02);
  EXPECT_TRUE(abs(1.0 - sd) < 0.02);

  EXPECT_FALSE(minimum < -4.1);
  EXPECT_FALSE(maximum > 4.1);
}

TEST_F(BlackBoxFixture, ComputeNewTunings) {
  gat::pid::types::Real kp, ki;

  gatpt::black::box::Parameters parameters;
  gatpt::black::box::Variables variables;

  parameters.Sd = 1.0;
  parameters.Kp = gat::types::make_range<gat::pid::types::Real>(1.0, 5.0);
  parameters.Ki = gat::types::make_range<gat::pid::types::Real>(0.001, 0.01);

  gatpt::black::box::initialize(parameters, variables);

  kp = variables.Kp;
  ki = variables.Ki;

  gatpt::black::box::seed(0.0);
  
  const int Count = 20000;

  for (int i = 0; i < Count; i++) {
    gatpt::black::box::compute::newtunings(parameters, variables);

    EXPECT_FALSE(kp == variables.Kp);
    EXPECT_FALSE(ki == variables.Ki);

    EXPECT_FALSE(variables.Kp < parameters.Kp.lowest);
    EXPECT_FALSE(variables.Kp > parameters.Kp.highest);
    EXPECT_FALSE(variables.Ki < parameters.Ki.lowest);
    EXPECT_FALSE(variables.Ki > parameters.Ki.highest);

  }
}