#pragma once
#include "esphome/components/cia402/cia402.h"

namespace esphome::tmcm1240 {
// Device registers and StallGuard only. Application positions and units belong in YAML.
class TMCM1240 : public cia402::CiA402Drive {
 public:
  using CiA402Drive::CiA402Drive;
  void setup() override;
  void dump_config() override;
  bool apply_configuration();
  bool apply_stallguard();
  bool save_configuration();
  bool seek_stall(int32_t distance, uint32_t margin);
  bool move_until_stop(int32_t distance, uint32_t margin);
  static uint8_t current_register(float amps);
  void set_run_current(float amps);
  void set_standby_current(float amps);
  void set_stallguard_threshold(int8_t value);
  void set_stallguard_min_velocity(uint32_t value);
  void set_consumer_timeout(uint16_t value) { this->consumer_timeout_ = value; }
  void set_master_node(uint8_t value) { this->master_node_ = value; }
  uint8_t get_driver_error_flags() const { return this->driver_error_flags_; }
  uint16_t get_stallguard_load() const { return this->stallguard_load_; }
  int8_t get_stallguard_threshold() const { return this->threshold_readback_; }
  uint32_t get_stallguard_min_velocity() const { return this->velocity_readback_; }
  bool is_stall_confirmed() const { return this->stall_confirmed_; }
  uint32_t get_reference_epoch() const { return this->reference_epoch_; }
  uint32_t get_heartbeat_age() const;
  uint32_t get_receive_age() const;
 protected:
  void append_stallguard_();
  void append_profile() override;
  void received_value(uint16_t index, uint32_t value) override;
  Step poll_step_(uint8_t index) const override;
  uint8_t poll_count_() const override { return 9; }
  void sequence_complete_() override;
  void finish_(cia402::DriveResult result) override;
  void on_node_offline() override;
  void on_node_emcy(const canopen::Emergency &emergency) override;
  void append_stall_sample_();
  uint32_t stallguard_min_velocity_{0};
  uint32_t reference_epoch_{0};
  uint32_t velocity_readback_{0};
  uint32_t stall_margin_{0};
  uint32_t last_heartbeat_ms_{0};
  uint32_t last_receive_ms_{0};
  int32_t seek_distance_{0};
  int32_t seek_target_{0};
  int32_t target_readback_{0};
  uint16_t consumer_timeout_{1500};
  uint16_t stallguard_load_{0};
  int8_t stallguard_threshold_{0};
  int8_t threshold_readback_{0};
  uint8_t run_current_{0};
  uint8_t standby_current_{0};
  uint8_t driver_error_flags_{0};
  uint8_t master_node_{0};
  uint8_t stopped_samples_{0};
  bool seeking_{false};
  bool sampling_{false};
  bool saw_motion_{false};
  bool stall_confirmed_{false};
  bool heartbeat_seen_{false};
  bool receive_seen_{false};
  bool require_stall_{true};
};
}  // namespace esphome::tmcm1240
