#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <Arduino.h>

#include <testboard/simple/rs485/rs485test/rs485test.ino>

class Rs485TestFixture : public ::testing::Test {
public:
  void SetUp() override {
    arduinomock = arduinoMockInstance();
    serial = serialMockInstance();
  }

  void TearDown() override {
    releaseSerialMock();
    releaseArduinoMock();
  }

  ArduinoMock* arduinomock;
  HardwareSerialMock* serial;
};

TEST_F(Rs485TestFixture, Concept) {
  uint16_t w = 0x1234;

  uint8_t lb = lowByte(w);
  uint8_t hb = highByte(w);

  EXPECT_EQ(0x34, lb);
  EXPECT_EQ(0x12, hb);

  uint8_t fhb = lb & 0x0f;
  uint8_t shb = lb >> 4;

  EXPECT_EQ(0x04, fhb);
  EXPECT_EQ(0x03, shb);
}

TEST_F(Rs485TestFixture, Setup) {
  EXPECT_CALL(*arduinomock, pinMode(PIN_RS485_TE, OUTPUT))
    .Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, pinMode(PIN_LED_RED_A, OUTPUT))
    .Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, pinMode(PIN_LED_RED_B, OUTPUT))
    .Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, pinMode(PIN_LED_BLUE_A, OUTPUT))
    .Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, pinMode(PIN_LED_BLUE_B, OUTPUT))
    .Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, pinMode(PIN_LED_GREEN, OUTPUT))
    .Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, pinMode(PIN_LED_YELLOW, OUTPUT))
    .Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, pinMode(PIN_BUTTON_A, INPUT_PULLUP))
    .Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, pinMode(PIN_BUTTON_B, INPUT_PULLUP))
    .Times(testing::Exactly(1));
  EXPECT_CALL(*serial, begin(RS485_BAUD)).Times(testing::Exactly(1));
  setup();
}

TEST_F(Rs485TestFixture, Loop) {
  EXPECT_CALL(*arduinomock, digitalWrite(PIN_LED_MODBUS_READ, HIGH));
  EXPECT_CALL(*serial, read()).WillOnce(testing::Return(-1));
  for (int i = 250; i >= 0; i -= 10) {
    EXPECT_CALL(*serial, read())
      .WillOnce(testing::Return(i));
  }
  EXPECT_CALL(*arduinomock, digitalWrite(PIN_LED_MODBUS_READ, LOW));
  loop();
}
