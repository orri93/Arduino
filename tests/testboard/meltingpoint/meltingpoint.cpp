#include <mutex>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <Arduino.h>

#include <testboard/meltingpoint/meltingpoint.ino>

namespace gm = ::gos::meltingpoint;
namespace gmd = ::gos::meltingpoint::display;

class MeltingPointFixture : public ::testing::Test {
public:
  void SetUp() override {
    arduinomock = arduinoMockInstance();
    if (::gos::meltingpoint::display::oled.U8g2 == nullptr) {
      ::gos::meltingpoint::display::oled.U8g2 = new U8g2;
    }
  }

  void TearDown() override {
    releaseArduinoMock();
    if (::gos::meltingpoint::display::oled.U8g2) {
      delete gmd::oled.U8g2;
      gmd::oled.U8g2 = nullptr;
    }
  }

  ArduinoMock* arduinomock;

  std::mutex mutex;
};

TEST_F(MeltingPointFixture, Setup) {
  EXPECT_CALL(*(gmd::oled.U8g2), begin()).
    Times(testing::AtLeast(1));

  setup();
}

TEST_F(MeltingPointFixture, Loop) {
  EXPECT_CALL(*arduinomock, millis)
    .Times(testing::AtLeast(1))
    .WillOnce(testing::Return(1));
  EXPECT_CALL(*(gmd::oled.U8g2), firstPage).Times(testing::AtLeast(1));
  EXPECT_CALL(*(gmd::oled.U8g2), setFont).Times(testing::AtLeast(1));
  EXPECT_CALL(*(gmd::oled.U8g2), drawStr).Times(testing::AtLeast(1));
  EXPECT_CALL(*(gmd::oled.U8g2), nextPage)
    .Times(testing::AtLeast(1))
    .WillOnce(testing::Return(0));
  loop();
}
