#include <mutex>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <Arduino.h>
#include <SPI.h>

#include <maxsensors/maxsensors.ino>

class MaxSensorsFixture : public ::testing::Test {
public:
  void SetUp() {
    arduinomock = arduinoMockInstance();
    serialmock = serialMockInstance();
    spimock = spiMockInstance();
  }

  void TearDown() {
    releaseSpiMock();
    releaseSerialMock();
    releaseArduinoMock();
  }

  ArduinoMock* arduinomock;
  SerialMock* serialmock;
  SpiMock* spimock;

  std::mutex mutex;
};

TEST_F(MaxSensorsFixture, Setup) {
  EXPECT_CALL(*arduinomock, pinMode(PIN_MAX_31865_CS, OUTPUT)).
    Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, digitalWrite(PIN_MAX_31865_CS, HIGH)).
    Times(testing::AtLeast(1));
  EXPECT_CALL(*arduinomock, digitalWrite(PIN_MAX_31865_CS, LOW)).
    Times(testing::AtLeast(1));
  EXPECT_CALL(*spimock, begin).Times(testing::AtLeast(1));
#ifdef SERIAL_BAUD
  EXPECT_CALL(*serialmock, begin((uint16_t)SERIAL_BAUD))
    .Times(testing::AtLeast(1));
#endif
  EXPECT_CALL(*serialmock, available)
    .Times(testing::AtLeast(1))
    .WillOnce(testing::Return(true));
  EXPECT_CALL(*serialmock, println(::testing::Matcher<const char*>(
    ::testing::StrEq(TEXT_MAX_SENSORS))))
    .Times(testing::AtLeast(1));

  setup();

  EXPECT_DOUBLE_EQ(0.0, value_31865);
}

TEST_F(MaxSensorsFixture, Loop) {
  EXPECT_CALL(*arduinomock, millis).
    Times(testing::Exactly(1)).
    WillOnce(testing::Return(1));
  EXPECT_CALL(*arduinomock, digitalWrite(PIN_MAX_31865_CS, LOW)).Times(::testing::AtLeast)
  loop();
}
