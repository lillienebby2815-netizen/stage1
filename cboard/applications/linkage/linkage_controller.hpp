#ifndef CBOARD_APPLICATIONS_LINKAGE_LINKAGE_CONTROLLER_HPP_
#define CBOARD_APPLICATIONS_LINKAGE_LINKAGE_CONTROLLER_HPP_

#include <cstdint>

namespace cboard
{

enum class LinkageMode : std::uint8_t
{
  kDisabled,
  kLinkage,
  kReset,
};

enum class ManualInputSource : std::uint8_t
{
  kNone,
  kMotorA,
  kMotorB,
};

enum class SwitchPosition : std::int8_t
{
  kDown = -1,
  kMiddle = 0,
  kUp = 1,
  kUnknown = 2,
};

struct LinkageConfig
{
  // All angles in this controller are radians. Motor encoder angles must be
  // supplied as continuous, unwrapped values by the motor feedback layer.
  float motor_a_to_yaw_ratio = 1.0F;
  float motor_b_ratio_down = 0.5F;
  float motor_b_ratio_middle = -1.0F;
  float motor_b_ratio_up = 3.0F;

  // During reset both motor R marks follow the C-board R mark at 1:1.
  // This sign/ratio is independent of the normal left-switch B ratio.
  float motor_b_reset_to_yaw_ratio = 1.0F;

  float manual_detection_threshold_rad = 0.02F;
  float target_reached_threshold_rad = 0.04F;
  float yaw_motion_threshold_rad_s = 0.01F;
};

struct LinkageInput
{
  float yaw_angle_rad = 0.0F;
  float yaw_rate_rad_s = 0.0F;
  float motor_a_angle_rad = 0.0F;
  float motor_b_angle_rad = 0.0F;

  SwitchPosition right_switch = SwitchPosition::kUnknown;
  SwitchPosition left_switch = SwitchPosition::kUnknown;

  bool remote_valid = false;
  bool imu_valid = false;
  bool motor_a_valid = false;
  bool motor_b_valid = false;
};

struct LinkageOutput
{
  LinkageMode mode = LinkageMode::kDisabled;
  bool motor_enable = false;
  bool reset_active = false;
  float motor_b_ratio = 0.0F;
  float motor_a_target_angle_rad = 0.0F;
  float motor_b_target_angle_rad = 0.0F;
  float yaw_reference_angle_rad = 0.0F;
  float motor_a_reference_angle_rad = 0.0F;
  float motor_b_reference_angle_rad = 0.0F;
  bool reset_reference_calibrated = false;
  ManualInputSource manual_input_source = ManualInputSource::kNone;
};

class LinkageController
{
public:
  explicit LinkageController(const LinkageConfig & config = LinkageConfig{});

  void reset();
  LinkageOutput update(const LinkageInput & input);

private:
  LinkageConfig config_;
  LinkageOutput output_;
  ManualInputSource manual_source_ = ManualInputSource::kNone;
  float manual_source_origin_angle_rad_ = 0.0F;
  float manual_source_origin_yaw_reference_rad_ = 0.0F;
  std::uint16_t manual_source_stationary_cycles_ = 0U;
  bool has_reference_ = false;
  bool startup_reset_reference_decided_ = false;
  bool reset_recalibration_required_ = false;
  bool reset_reference_calibrated_ = false;
  float reset_reference_yaw_angle_rad_ = 0.0F;
  float reset_reference_motor_a_angle_rad_ = 0.0F;
  float reset_reference_motor_b_angle_rad_ = 0.0F;
  float previous_yaw_sensor_angle_rad_ = 0.0F;
  float yaw_unwrapped_angle_rad_ = 0.0F;
  bool yaw_sensor_initialized_ = false;
  float previous_yaw_angle_rad_ = 0.0F;
  float previous_motor_a_angle_rad_ = 0.0F;
  float previous_motor_b_angle_rad_ = 0.0F;
  float previous_motor_a_target_angle_rad_ = 0.0F;
  float previous_motor_b_target_angle_rad_ = 0.0F;
  float previous_motor_b_ratio_ = 0.0F;
  bool motor_b_ratio_initialized_ = false;

  static LinkageMode mode_from_switch(SwitchPosition right_switch);
  float ratio_from_switch(SwitchPosition left_switch) const;
  bool inputs_valid(const LinkageInput & input) const;
  bool config_valid() const;
  void update_yaw_tracking(const LinkageInput & input);
  void capture_reference(const LinkageInput & input);
  void capture_reset_reference(const LinkageInput & input);
  void update_measurement_history(const LinkageInput & input);
  void make_disabled_output(const LinkageInput & input);
  void make_reset_output(const LinkageInput & input);
  void update_linkage_output(const LinkageInput & input, float motor_b_ratio);
};

}  // namespace cboard

#endif  // CBOARD_APPLICATIONS_LINKAGE_LINKAGE_CONTROLLER_HPP_
