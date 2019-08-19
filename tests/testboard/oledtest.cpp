#include <mutex>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <Arduino.h>

#include <testboard/oledtest/oledtest.ino>

class OledTestFixture : public ::testing::Test {
public:
  void SetUp() override {
    arduinomock = arduinoMockInstance();
    if (oled.U8g2 == nullptr) {
      oled.U8g2 = new U8g2;
    }
  }

  void TearDown() override {
    releaseArduinoMock();
    if (oled.U8g2) {
      delete oled.U8g2;
      oled.U8g2 = nullptr;
    }
  }

  ArduinoMock* arduinomock;

  std::mutex mutex;
};

TEST_F(OledTestFixture, Setup) {
  EXPECT_CALL(*(oled.U8g2), begin()).
    Times(testing::AtLeast(1));

  setup();
}

TEST_F(OledTestFixture, Loop) {
  EXPECT_CALL(*arduinomock, millis)
    .Times(testing::AtLeast(1))
    .WillOnce(testing::Return(1));
  EXPECT_CALL(*(oled.U8g2), firstPage).Times(testing::AtLeast(1));
  EXPECT_CALL(*(oled.U8g2), setFont).Times(testing::AtLeast(1));
  EXPECT_CALL(*(oled.U8g2), drawStr).Times(testing::AtLeast(1));
  EXPECT_CALL(*(oled.U8g2), nextPage)
    .Times(testing::AtLeast(1))
    .WillOnce(testing::Return(0));
  loop();
}
