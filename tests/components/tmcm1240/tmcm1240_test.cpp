#include <gtest/gtest.h>
#include <map>
#include "esphome/components/tmcm1240/tmcm1240.h"
#include "esphome/core/application.h"
#include "../canopen/fake_canbus.h"

namespace esphome::tmcm1240::testing {
class ProbeMotor : public TMCM1240 {
 public:
  using TMCM1240::TMCM1240;
  void tick() {
    LoopBlockingGuard guard(nullptr, nullptr, millis());
    this->last_request_ms_ -= 50;
    this->loop();
  }
  void expire() { this->started_ms_ -= this->operation_timeout_ms_; }
};

class MotorTest : public ::testing::Test {
 public:
  canopen::testing::FakeCanbus bus;
  canopen::testing::TestHub client;
  ProbeMotor motor{&client, 32};
  std::map<uint16_t, uint32_t> objects{{0x6041, 0x27}, {0x6064, 10000}, {0x606C, 0}};
  std::vector<cia402::DriveResult> results;
  size_t processed{0};
  unsigned samples{0};
  bool moving{false};
  bool arrive{false};
  void SetUp() override {
    client.set_canbus(&bus);
    client.monitor_node(32, 60000);
    client.setup();
    client.loop();
    client.frame(0x720, {5});
    bus.sent.clear();
    motor.set_run_current(0.5f);
    motor.set_standby_current(0.25f);
    motor.set_stallguard_threshold(8);
    motor.set_stallguard_min_velocity(2800);
    motor.set_profile_velocity(3000);
    motor.set_acceleration(5000);
    motor.set_deceleration(5000);
    motor.add_on_result_callback([this](cia402::DriveResult result) { results.push_back(result); });
  }
  void step() {
    motor.tick();
    client.loop();
    while (processed < bus.sent.size()) {
      const auto frame = bus.sent[processed++];
      if (frame.can_id != 0x620 || frame.data[0] == 0x80)
        continue;
      const uint16_t index = frame.data[1] | (frame.data[2] << 8);
      const bool read = frame.data[0] == 0x40;
      uint32_t value = encode_uint32(frame.data[7], frame.data[6], frame.data[5], frame.data[4]);
      if (!read) {
        objects[index] = value;
        if (index == 0x6060)
          objects[0x6061] = value;
        if (index == 0x6040 && value == 0x3F) {
          moving = true;
          objects[0x6041] = 0x1027;
          objects[0x606C] = static_cast<uint32_t>(-3000);
        }
        if (index == 0x6040 && value == 15)
          objects[0x6041] &= ~0x1000;
      } else {
        if (moving && index == 0x6064) {
          ++samples;
          objects[0x6064] = arrive ? objects[0x607A] : 7000;
          if (samples >= 2 || arrive)
            objects[0x606C] = 0;
          if (arrive)
            objects[0x6041] |= 0x400;
        }
        value = objects[index];
      }
      const uint8_t size = index == 0x6041 || index == 0x2101 ? 2 :
          (index == 0x6061 || index == 0x2102 || index == 0x209E ? 1 : 4);
      const uint8_t command = read ? 0x43 | ((4 - size) << 2) : 0x60;
      client.frame(0x5A0, {command, frame.data[1], frame.data[2], frame.data[3], static_cast<uint8_t>(value),
                          static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value >> 16),
                          static_cast<uint8_t>(value >> 24)});
    }
  }
  void run() {
    for (unsigned i = 0; i < 300 && motor.busy(); i++)
      step();
  }
};

TEST(TMCM1240Current, InvalidCurrentsAreBounded) {
  EXPECT_EQ(TMCM1240::current_register(NAN), 0);
  EXPECT_EQ(TMCM1240::current_register(-1), 0);
  EXPECT_EQ(TMCM1240::current_register(2.1f), 0);
  EXPECT_EQ(TMCM1240::current_register(0.5f) % 8, 0);
}
TEST_F(MotorTest, ConfigurationDoesNotEnableOrSave) {
  ASSERT_TRUE(motor.apply_configuration());
  run();
  ASSERT_FALSE(results.empty());
  EXPECT_EQ(results.back(), cia402::DriveResult::DRIVE_RESULT_SUCCESS);
  EXPECT_EQ(objects[0x2003], TMCM1240::current_register(0.5f));
  EXPECT_EQ(objects[0x20A7], 1);
  EXPECT_EQ(objects.count(0x1010), 0);
  EXPECT_EQ(objects.count(0x6040), 0);
}
TEST_F(MotorTest, SaveRequiresDisabledDrive) {
  ASSERT_TRUE(motor.save_configuration());
  run();
  EXPECT_EQ(objects.count(0x1010), 0);
  EXPECT_EQ(results.front(), cia402::DriveResult::DRIVE_RESULT_REJECTED);
}
TEST_F(MotorTest, ConfirmedStallDoesNotZeroPosition) {
  ASSERT_TRUE(motor.seek_stall(-5000, 200));
  run();
  ASSERT_FALSE(motor.busy());
  EXPECT_TRUE(motor.is_stall_confirmed());
  EXPECT_GE(samples, 3u);
  EXPECT_EQ(objects.count(0x6098), 0);
  EXPECT_EQ(objects[0x6064], 7000);
}
TEST_F(MotorTest, FullTravelDoesNotCountAsHomingStall) {
  arrive = true;
  ASSERT_TRUE(motor.seek_stall(-5000, 200));
  run();
  EXPECT_FALSE(motor.is_stall_confirmed());
  EXPECT_EQ(objects.count(0x6098), 0);
  EXPECT_NE(results.front(), cia402::DriveResult::DRIVE_RESULT_SUCCESS);
}
TEST_F(MotorTest, DriverErrorCannotCountAsStall) {
  objects[0x2102] = 4;
  ASSERT_TRUE(motor.seek_stall(-5000, 200));
  run();
  EXPECT_FALSE(motor.is_stall_confirmed());
  EXPECT_EQ(objects.count(0x6098), 0);
}
TEST_F(MotorTest, NodeRestartInvalidatesReferenceAndCancelsMotion) {
  ASSERT_TRUE(motor.seek_stall(-5000, 200));
  step();
  const auto epoch = motor.get_reference_epoch();
  client.frame(0x720, {0});
  EXPECT_GT(motor.get_reference_epoch(), epoch);
  EXPECT_FALSE(motor.busy());
  EXPECT_FALSE(motor.is_stall_confirmed());
}
TEST_F(MotorTest, SeekBelowActivationSpeedIsRejected) {
  motor.set_profile_velocity(2000);
  EXPECT_FALSE(motor.seek_stall(-5000, 200));
  EXPECT_FALSE(motor.busy());
}
TEST_F(MotorTest, StallEmergencyAloneDoesNotConfirmAStall) {
  ASSERT_TRUE(motor.seek_stall(-5000, 200));
  client.frame(0xA0, {0x30, 0x32, 0, 0, 0, 0, 0, 0});
  EXPECT_TRUE(motor.busy());
  EXPECT_FALSE(motor.is_stall_confirmed());
  EXPECT_TRUE(results.empty());
}
TEST_F(MotorTest, OtherEmergencyFailsSeek) {
  ASSERT_TRUE(motor.seek_stall(-5000, 200));
  client.frame(0xA0, {0, 0xFF, 0, 0, 0, 0, 0, 0});
  EXPECT_FALSE(motor.is_stall_confirmed());
  ASSERT_FALSE(results.empty());
  EXPECT_EQ(results.front(), cia402::DriveResult::DRIVE_RESULT_FAULT);
}
}  // namespace esphome::tmcm1240::testing
