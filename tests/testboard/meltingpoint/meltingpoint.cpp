#include <mutex>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <Arduino.h>

#include <EEPROM.h>
#include <SPI.h>

#include <mock/ModbusSlave.h>

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
    serial = serialMockInstance();
    eeprom = EEPROMMockInstance();
    spi = spiMockInstance();

    ::gos::meltingpoint::modbus::slave.createcoils(1);
    ::gos::meltingpoint::modbus::slave.createregisters(16);
  }

  void TearDown() override {
    releaseSpiMock();
    releaseEEPROMMock();
    releaseSerialMock();
    releaseArduinoMock();
    if (::gos::meltingpoint::display::oled.U8g2) {
      delete gmd::oled.U8g2;
      gmd::oled.U8g2 = nullptr;
    }
  }

  ArduinoMock* arduinomock;
  HardwareSerialMock* serial;
  EEPROMMock* eeprom;
  SpiMock* spi;

  std::mutex mutex;
};

TEST_F(MeltingPointFixture, Setup) {
  EXPECT_CALL(*(gmd::oled.U8g2), begin()).
    Times(testing::AtLeast(1));
  EXPECT_CALL(*arduinomock, pinMode).Times(testing::AtLeast(4));
  EXPECT_CALL(*spi, begin).Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, digitalWrite).Times(testing::AtLeast(1)); // Max6675
  EXPECT_CALL(*serial, begin(MODBUS_BAUD)).Times(testing::Exactly(1));
  EXPECT_CALL(*arduinomock, delay).Times(testing::AtLeast(1));

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
  EXPECT_CALL(*arduinomock, analogRead).Times(testing::AtLeast(1));
  EXPECT_CALL(*arduinomock, digitalWrite).Times(testing::AtLeast(2));
  loop();
}

TEST_F(MeltingPointFixture, WriteHoldingRegisters) {
  uint8_t modbusresult;
  ::gos::meltingpoint::modbus::binding::create();

  gm::type::Real number = 93.11;
  gm::modbus::slave.setregisters<gm::type::Real>(0, number);

  modbusresult = ::gos::meltingpoint::modbus::write_holding_registers(
    16,   // Function
    3,    // Address of Kp
    2);   // Length
  EXPECT_EQ(STATUS_OK, modbusresult);
  EXPECT_FLOAT_EQ(number, gm::pid::parameter.Kp);

  gatl::binding::testing::clean<bool, uint16_t, uint8_t>(
    gm::modbus::binding::coils);

  gatl::binding::testing::clean<gm::type::Output, uint16_t, uint8_t>(
    gm::modbus::binding::input::output);
  gatl::binding::testing::clean<gm::type::Real, uint16_t, uint8_t>(
    gm::modbus::binding::input::sensor);

  gatl::binding::change::aware::testing::clean<
    gm::type::Output, uint16_t, uint8_t>(
    gm::modbus::binding::holding::manual);
  gatl::binding::change::aware::testing::clean<
    gm::type::Real, uint16_t, uint8_t>(
      gm::modbus::binding::holding::pid);
}

