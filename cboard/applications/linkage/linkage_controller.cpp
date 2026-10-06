#include "linkage_controller.hpp"

#include <cmath>

namespace
{
constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;
constexpr float kMinimumNonzeroRatio = 1.0e-4F;

float wrap_angle_rad(float angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= kTwoPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += kTwoPi;
  }
  return angle_rad;
}

float absolute_value(float value) { return value >= 0.0F ? value : -value; }
}  // namespace

namespace cboard
{

LinkageController::LinkageController(const LinkageConfig & config) : config_(config) { reset(); }

void LinkageController::reset()
{
  output_ = {};
  has_reference_ = false;
  previous_yaw_sensor_angle_rad_ = 0.0F;
  yaw_unwrapped_angle_rad_ = 0.0F;
  yaw_sensor_initialized_ = false;
  previous_yaw_angle_rad_ = 0.0F;
  previous_motor_a_angle_rad_ = 0.0F;
  previous_motor_b_angle_rad_ = 0.0F;
  previous_motor_a_target_angle_rad_ = 0.0F;
  previous_motor_b_target_angle_rad_ = 0.0F;
  previous_motor_b_ratio_ = 0.0F;
  motor_b_ratio_initialized_ = false;
}

LinkageOutput LinkageController::update(const LinkageInput & input)
{
  update_yaw_tracking(input);
  const LinkageMode requested_mode = mode_from_switch(input.right_switch);

  // Any missing safety-critical input forces zero motor effort. The motor
  // layer must translate motor_enable=false into a zero-current/disable command.
  if (
    !inputs_valid(input) || requested_mode == LinkageMode::kDisabled ||
    (requested_mode == LinkageMode::kLinkage && input.left_switch == SwitchPosition::kUnknown)) {
    make_disabled_output(input);
    if (inputs_valid(input)) {
      update_measurement_history(input);
    }
    return output_;
  }

  if (requested_mode != output_.mode) {
    if (requested_mode == LinkageMode::kLinkage) {
      // Enter linkage without a target jump. The current physical positions
      // become the new reference point.
      capture_reference(input);
    }
    output_.mode = requested_mode;
  }

  if (requested_mode == LinkageMode::kReset) {
    make_reset_output(input);
    update_measurement_history(input);
    return output_;
  }

  const float motor_b_ratio = ratio_from_switch(input.left_switch);
  update_linkage_output(input, motor_b_ratio);
  update_measurement_history(input);
  return output_;
}

LinkageMode LinkageController::mode_from_switch(SwitchPosition right_switch)
{
  switch (right_switch) {
    case SwitchPosition::kMiddle:
      return LinkageMode::kLinkage;
    case SwitchPosition::kUp:
      return LinkageMode::kReset;
    case SwitchPosition::kDown:
    case SwitchPosition::kUnknown:
    default:
      return LinkageMode::kDisabled;
  }
}

float LinkageController::ratio_from_switch(SwitchPosition left_switch) const
{
  switch (left_switch) {
    case SwitchPosition::kDown:
      return config_.motor_b_ratio_down;
    case SwitchPosition::kMiddle:
      return config_.motor_b_ratio_middle;
    case SwitchPosition::kUp:
      return config_.motor_b_ratio_up;
    case SwitchPosition::kUnknown:
    default:
      return 0.0F;
  }
}

bool LinkageController::inputs_valid(const LinkageInput & input) const
{
  return config_valid() && input.remote_valid && input.imu_valid && input.motor_a_valid &&
         input.motor_b_valid && std::isfinite(input.yaw_angle_rad) &&
         std::isfinite(input.motor_a_angle_rad) && std::isfinite(input.motor_b_angle_rad);
}

bool LinkageController::config_valid() const
{
  return std::isfinite(config_.motor_a_to_yaw_ratio) && std::isfinite(config_.motor_b_ratio_down) &&
         std::isfinite(config_.motor_b_ratio_middle) && std::isfinite(config_.motor_b_ratio_up) &&
         std::isfinite(config_.motor_a_reset_angle_rad) &&
         std::isfinite(config_.motor_b_reset_angle_rad) &&
         std::isfinite(config_.manual_detection_threshold_rad) &&
         std::isfinite(config_.target_reached_threshold_rad) &&
         std::isfinite(config_.yaw_motion_threshold_rad) &&
         config_.manual_detection_threshold_rad > 0.0F &&
         config_.target_reached_threshold_rad > 0.0F && config_.yaw_motion_threshold_rad >= 0.0F;
}

void LinkageController::update_yaw_tracking(const LinkageInput & input)
{
  if (!input.imu_valid || !std::isfinite(input.yaw_angle_rad)) {
    yaw_sensor_initialized_ = false;
    return;
  }

  if (!yaw_sensor_initialized_) {
    previous_yaw_sensor_angle_rad_ = input.yaw_angle_rad;
    yaw_unwrapped_angle_rad_ = input.yaw_angle_rad;
    previous_yaw_angle_rad_ = yaw_unwrapped_angle_rad_;
    yaw_sensor_initialized_ = true;
    return;
  }

  // Unwrap the IMU heading across the -pi/+pi boundary so a full rotation
  // produces a continuous motor target instead of a 2*pi jump.
  yaw_unwrapped_angle_rad_ += wrap_angle_rad(input.yaw_angle_rad - previous_yaw_sensor_angle_rad_);
  previous_yaw_sensor_angle_rad_ = input.yaw_angle_rad;
}

void LinkageController::capture_reference(const LinkageInput & input)
{
  output_.yaw_reference_angle_rad = yaw_unwrapped_angle_rad_;
  output_.motor_a_reference_angle_rad = input.motor_a_angle_rad;
  output_.motor_b_reference_angle_rad = input.motor_b_angle_rad;
  has_reference_ = true;
  previous_yaw_angle_rad_ = yaw_unwrapped_angle_rad_;
  previous_motor_a_angle_rad_ = input.motor_a_angle_rad;
  previous_motor_b_angle_rad_ = input.motor_b_angle_rad;
  previous_motor_a_target_angle_rad_ = input.motor_a_angle_rad;
  previous_motor_b_target_angle_rad_ = input.motor_b_angle_rad;
}

void LinkageController::update_measurement_history(const LinkageInput & input)
{
  previous_yaw_angle_rad_ = yaw_unwrapped_angle_rad_;
  previous_motor_a_angle_rad_ = input.motor_a_angle_rad;
  previous_motor_b_angle_rad_ = input.motor_b_angle_rad;
}

void LinkageController::make_disabled_output(const LinkageInput & input)
{
  output_.mode = LinkageMode::kDisabled;
  output_.motor_enable = false;
  output_.reset_active = false;
  output_.motor_b_ratio = 0.0F;

  // Keeping targets at measured positions prevents a later motor layer from
  // creating a position jump if it temporarily ignores motor_enable.
  output_.motor_a_target_angle_rad = input.motor_a_angle_rad;
  output_.motor_b_target_angle_rad = input.motor_b_angle_rad;
}

void LinkageController::make_reset_output(const LinkageInput & input)
{
  output_.mode = LinkageMode::kReset;
  // Do not command real motors toward a guessed zero. The mechanical arrow
  // alignment must be calibrated before reset output is enabled.
  output_.motor_enable = config_.reset_reference_calibrated;
  output_.motor_b_ratio = 0.0F;
  output_.motor_a_target_angle_rad = config_.motor_a_reset_angle_rad;
  output_.motor_b_target_angle_rad = config_.motor_b_reset_angle_rad;
  output_.reset_active =
    output_.motor_enable &&
    (absolute_value(input.motor_a_angle_rad - output_.motor_a_target_angle_rad) >
       config_.target_reached_threshold_rad ||
     absolute_value(input.motor_b_angle_rad - output_.motor_b_target_angle_rad) >
       config_.target_reached_threshold_rad);
}

void LinkageController::update_linkage_output(const LinkageInput & input, float motor_b_ratio)
{
  if (!has_reference_) {
    capture_reference(input);
  }

  const float yaw_step_rad = yaw_unwrapped_angle_rad_ - previous_yaw_angle_rad_;
  const float motor_a_step_rad = input.motor_a_angle_rad - previous_motor_a_angle_rad_;
  const float motor_b_step_rad = input.motor_b_angle_rad - previous_motor_b_angle_rad_;
  const float yaw_offset_rad = yaw_unwrapped_angle_rad_ - output_.yaw_reference_angle_rad;

  // Changing the left switch changes the B ratio. Re-anchor B at its current
  // measured position so selecting another ratio never creates a target jump.
  if (
    !motor_b_ratio_initialized_ ||
    absolute_value(motor_b_ratio - previous_motor_b_ratio_) > kMinimumNonzeroRatio) {
    output_.motor_b_reference_angle_rad =
      input.motor_b_angle_rad - yaw_offset_rad * motor_b_ratio;
    motor_b_ratio_initialized_ = true;
    previous_motor_b_ratio_ = motor_b_ratio;
  }

  const bool yaw_is_quiet = absolute_value(yaw_step_rad) <= config_.yaw_motion_threshold_rad;
  const bool motor_a_was_settled =
    absolute_value(previous_motor_a_angle_rad_ - previous_motor_a_target_angle_rad_) <=
    config_.target_reached_threshold_rad;
  const bool motor_b_was_settled =
    absolute_value(previous_motor_b_angle_rad_ - previous_motor_b_target_angle_rad_) <=
    config_.target_reached_threshold_rad;

  // When Yaw is steady and a motor moves away from its settled target, treat
  // the larger movement as manual input. The reference point then moves with
  // that motor, so the other motor follows without changing the Yaw reference.
  if (yaw_is_quiet && motor_a_was_settled && motor_b_was_settled) {
    const bool motor_a_moved =
      absolute_value(motor_a_step_rad) > config_.manual_detection_threshold_rad;
    const bool motor_b_moved =
      absolute_value(motor_b_step_rad) > config_.manual_detection_threshold_rad;

    if (
      motor_a_moved &&
      (!motor_b_moved || absolute_value(motor_a_step_rad) >= absolute_value(motor_b_step_rad))) {
      output_.motor_a_reference_angle_rad += motor_a_step_rad;
      output_.motor_b_reference_angle_rad += motor_b_ratio * motor_a_step_rad;
    } else if (motor_b_moved && absolute_value(motor_b_ratio) > kMinimumNonzeroRatio) {
      output_.motor_b_reference_angle_rad += motor_b_step_rad;
      output_.motor_a_reference_angle_rad += motor_b_step_rad / motor_b_ratio;
    }
  }

  output_.mode = LinkageMode::kLinkage;
  output_.motor_enable = true;
  output_.reset_active = false;
  output_.motor_b_ratio = motor_b_ratio;
  output_.motor_a_target_angle_rad =
    output_.motor_a_reference_angle_rad + yaw_offset_rad * config_.motor_a_to_yaw_ratio;
  output_.motor_b_target_angle_rad =
    output_.motor_b_reference_angle_rad + yaw_offset_rad * motor_b_ratio;
  previous_motor_a_target_angle_rad_ = output_.motor_a_target_angle_rad;
  previous_motor_b_target_angle_rad_ = output_.motor_b_target_angle_rad;
}

}  // namespace cboard
