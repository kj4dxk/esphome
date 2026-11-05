#include "tmcm1240.h"
#include <algorithm>
#include <cmath>
#include <numbers>
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome::tmcm1240 {
ESPHOME_LOG_TAG(TAG, "tmcm1240");
using cia402::DriveResult;

void TMCM1240::setup() {
  this->get_canopen()->add_on_heartbeat_callback([this](uint8_t node, canopen::NMTState state) {
    if (node == this->get_node_id()) {
      this->heartbeat_seen_ = true;
      this->last_heartbeat_ms_ = App.get_loop_component_start_time();
    }
  });
  CiA402Drive::setup();
}

uint32_t TMCM1240::get_heartbeat_age() const {
  return this->heartbeat_seen_ ? millis() - this->last_heartbeat_ms_ : UINT32_MAX;
}
uint32_t TMCM1240::get_receive_age() const {
  return this->receive_seen_ ? millis() - this->last_receive_ms_ : UINT32_MAX;
}
void TMCM1240::dump_config() {
  ESP_LOGCONFIG(TAG, "TMCM-1240 node %u", this->get_node_id());
  ESP_LOGCONFIG(TAG, "  Run/standby current registers: %u/%u", this->run_current_, this->standby_current_);
  LOG_UPDATE_INTERVAL(this);
}
uint8_t TMCM1240::current_register(float amps) {
  if (!std::isfinite(amps) || amps < 0.072f || amps > 2.0f)
    return 0;
  const int32_t level = std::clamp<int32_t>(
      static_cast<int32_t>(std::lround(amps * 32.0f * std::numbers::sqrt2_v<float> / 3.250f)), 1, 32);
  return static_cast<uint8_t>((level - 1) * 8);
}
void TMCM1240::set_run_current(float amps) {
  if (std::isfinite(amps) && amps >= 0.072f && amps <= 2.0f)
    this->run_current_ = current_register(amps);
}
void TMCM1240::set_standby_current(float amps) {
  if (std::isfinite(amps) && amps >= 0.072f && amps <= 2.0f)
    this->standby_current_ = current_register(amps);
}
void TMCM1240::set_stallguard_threshold(int8_t value) {
  if (!this->seeking_ && value >= -63 && value <= 63)
    this->stallguard_threshold_ = value;
}
void TMCM1240::set_stallguard_min_velocity(uint32_t value) {
  if (!this->seeking_ && value > 0 && value <= INT32_MAX)
    this->stallguard_min_velocity_ = value;
}
void TMCM1240::append_stallguard_() {
  this->write_(0x209C, 0, 1);
  this->write_(0x209D, 0, 1);
  this->write_(0x209E, static_cast<uint8_t>(this->stallguard_threshold_), 1);
  this->write_(0x20A7, 1, 1);
  this->write_(0x20A4, this->stallguard_min_velocity_, 4);
}
bool TMCM1240::apply_stallguard() {
  if (!this->begin_())
    return false;
  this->read_(0x606C, 4, UINT32_MAX, 0);
  this->append_stallguard_();
  return true;
}
bool TMCM1240::apply_configuration() {
  if (this->standby_current_ > this->run_current_ || !this->begin_())
    return false;
  this->read_(0x6041, 2, 8, 0);
  this->read_(0x606C, 4, UINT32_MAX, 0);
  this->write_(0x1017, 500, 2);
  this->write_(0x1016, (static_cast<uint32_t>(this->master_node_) << 16) | this->consumer_timeout_, 4, 1);
  this->write_(0x1029, 0, 1, 1);
  this->write_(0x2003, this->run_current_, 1);
  this->write_(0x2004, this->standby_current_, 1);
  this->write_(0x20B0, 0, 4);
  this->write_(0x20B1, 0, 1);
  this->write_(0x2005, 3, 4);
  this->append_stallguard_();
  this->append_profile();
  return true;
}
bool TMCM1240::save_configuration() {
  if (!this->begin_())
    return false;
  this->read_(0x6041, 2, 0x4F, 0x40);
  this->read_(0x606C, 4, UINT32_MAX, 0);
  for (uint8_t sub : {1, 2, 4})
    this->write_(0x1010, 0x65766173, 4, sub);
  return true;
}
void TMCM1240::append_profile() {
  CiA402Drive::append_profile();
  this->write_(0x2011, this->acceleration_, 4);
  this->write_(0x2013, this->deceleration_, 4);
}
cia402::CiA402Drive::Step TMCM1240::poll_step_(uint8_t index) const {
  if (index < 5)
    return CiA402Drive::poll_step_(index);
  static constexpr uint16_t indices[] = {0x2102, 0x2101, 0x209E, 0x20A4};
  static constexpr uint8_t sizes[] = {1, 2, 1, 4};
  return {0, 0, indices[index - 5], 0, sizes[index - 5], StepKind::STEP_KIND_READ};
}
void TMCM1240::received_value(uint16_t index, uint32_t value) {
  this->last_receive_ms_ = App.get_loop_component_start_time();
  this->receive_seen_ = true;
  switch (index) {
    case 0x2102: this->driver_error_flags_ = value; break;
    case 0x2101: this->stallguard_load_ = value; break;
    case 0x209E: this->threshold_readback_ = sign_extend<8>(value); break;
    case 0x20A4: this->velocity_readback_ = value; break;
    case 0x607A: this->target_readback_ = sign_extend<32>(value); break;
    default: break;
  }
  CiA402Drive::received_value(index, value);
}
bool TMCM1240::seek_stall(int32_t distance, uint32_t margin) {
  if (distance == 0 || margin == 0 || static_cast<int64_t>(margin) >= std::abs(static_cast<int64_t>(distance)) ||
      this->profile_velocity_ <= this->stallguard_min_velocity_ || !this->begin_())
    return false;
  this->stall_confirmed_ = false;
  this->require_stall_ = true;
  this->seeking_ = true;
  this->sampling_ = false;
  this->saw_motion_ = false;
  this->stopped_samples_ = 0;
  this->seek_distance_ = distance;
  this->stall_margin_ = margin;
  this->append_stallguard_();
  this->append_move_(distance, true);
  // Replace target-arrival checks: StallGuard intentionally stops before the requested target.
  this->step_count_ -= 3;
  return true;
}
bool TMCM1240::move_until_stop(int32_t distance, uint32_t margin) {
  if (distance == 0)
    return this->refresh();
  const uint32_t length = static_cast<uint32_t>(std::abs(static_cast<int64_t>(distance)));
  margin = std::min(margin, length > 1 ? length - 1 : 1);
  if (length == 1)
    return this->move_by(distance);
  if (!this->seek_stall(distance, margin))
    return false;
  this->require_stall_ = false;
  return true;
}
void TMCM1240::append_stall_sample_() {
  this->step_count_ = 0;
  this->step_index_ = 0;
  this->read_(0x6041, 2);
  this->read_(0x6064, 4);
  this->read_(0x606C, 4);
  this->read_(0x2102, 1);
  this->read_(0x2101, 2);
  this->read_(0x209E, 1);
  this->read_(0x20A4, 4);
  this->read_(0x607A, 4);
}
void TMCM1240::sequence_complete_() {
  if (!this->seeking_) {
    CiA402Drive::sequence_complete_();
    return;
  }
  if (!this->sampling_) {
    for (uint8_t i = 0; i < this->step_count_; i++)
      if (this->steps_[i].index == 0x607A)
        this->seek_target_ = sign_extend<32>(this->steps_[i].value);
    this->sampling_ = true;
  } else {
    const int64_t direction = this->seek_distance_ > 0 ? 1 : -1;
    if (direction * this->velocity_ > this->stallguard_min_velocity_)
      this->saw_motion_ = true;
    const int64_t remaining = direction * (static_cast<int64_t>(this->seek_target_) - this->position_);
    if (!this->require_stall_ && this->velocity_ == 0 && (this->statusword_ & 0x400) &&
        std::abs(remaining) <= this->stall_margin_ && this->driver_error_flags_ == 0) {
      CiA402Drive::sequence_complete_();
      return;
    }
    const bool confirmed = this->saw_motion_ && this->velocity_ == 0 && remaining > this->stall_margin_ &&
                           !(this->statusword_ & 0x408) && this->driver_error_flags_ == 0 &&
                           this->stallguard_load_ == 0 && this->threshold_readback_ == this->stallguard_threshold_ &&
                           this->velocity_readback_ == this->stallguard_min_velocity_ &&
                           this->target_readback_ == this->seek_target_;
    this->stopped_samples_ = confirmed ? this->stopped_samples_ + 1 : 0;
    if (this->stopped_samples_ >= 2) {
      this->stall_confirmed_ = true;
      this->seeking_ = false;
      this->step_count_ = this->step_index_ = 0;
      this->write_(0x6040, 0x10F, 2);
      this->read_(0x606C, 4, UINT32_MAX, 0, true);
      return;
    }
    if ((this->statusword_ & 8) || this->driver_error_flags_ != 0 ||
        (this->velocity_ == 0 && remaining <= this->stall_margin_)) {
      this->fail_(DriveResult::DRIVE_RESULT_FAULT);
      return;
    }
  }
  this->append_stall_sample_();
}
void TMCM1240::finish_(DriveResult result) {
  this->seeking_ = false;
  this->sampling_ = false;
  if (result != DriveResult::DRIVE_RESULT_SUCCESS)
    this->stall_confirmed_ = false;
  CiA402Drive::finish_(result);
}
void TMCM1240::on_node_offline() {
  this->reference_epoch_++;
  this->stall_confirmed_ = false;
  CiA402Drive::on_node_offline();
}
void TMCM1240::on_node_emcy(const canopen::Emergency &emergency) {
  // Firmware manual: 0x3230 is StallGuard. Still require every readback check before accepting a stall.
  if (this->seeking_ && emergency.error_code == 0x3230) {
    this->last_emcy_ = emergency.error_code;
    return;
  }
  CiA402Drive::on_node_emcy(emergency);
}
}  // namespace esphome::tmcm1240
