#include "linkage_controller.hpp"

#include <cmath>

namespace
{
constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;
constexpr float kMinimumNonzeroRatio = 1.0e-4F;
constexpr float kEncoderResolutionRad = kTwoPi / 8192.0F;
// Keep the detected source latched while the ratio follower settles. The
// linkage task runs at 1 kHz, so this is 300 ms after hand motion stops.
constexpr std::uint16_t kManualSourceQuietCycles = 300U;

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
  manual_source_ = ManualInputSource::kNone;
  manual_source_origin_angle_rad_ = 0.0F;
  manual_source_origin_yaw_reference_rad_ = 0.0F;
  manual_source_stationary_cycles_ = 0U;
  has_reference_ = false;
  startup_reset_reference_decided_ = false;
  reset_recalibration_required_ = false;
  reset_reference_calibrated_ = false;
  reset_reference_yaw_angle_rad_ = 0.0F;
  reset_reference_motor_a_angle_rad_ = 0.0F;
  reset_reference_motor_b_angle_rad_ = 0.0F;
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

  // BMI088 yaw is gyro-integrated. If it becomes invalid, the estimator may
  // restart from zero, so discard the running linkage reference and inhibit
  // reset until a fresh, explicitly aligned startup calibration.
  if (!input.imu_valid) {
    reset_reference_calibrated_ = false;
    reset_recalibration_required_ = true;
    has_reference_ = false;
    motor_b_ratio_initialized_ = false;
    manual_source_ = ManualInputSource::kNone;
    make_disabled_output(input);
    return output_;
  }

  // Capture mechanical R-mark alignment once at startup, with valid feedback
  // and the right switch down. Otherwise reset remains inhibited until reboot.
  if (inputs_valid(input) &&
      (!startup_reset_reference_decided_ || reset_recalibration_required_)) {
    if (requested_mode == LinkageMode::kDisabled) {
      capture_reset_reference(input);
      reset_recalibration_required_ = false;
      startup_reset_reference_decided_ = true;
    } else if (!reset_recalibration_required_) {
      startup_reset_reference_decided_ = true;
    }
  }

  // Any missing safety-critical input forces zero motor effort. The motor
  // layer must translate motor_enable=false into a zero-current/disable command.
  if (
    !inputs_valid(input) || requested_mode == LinkageMode::kDisabled ||
    (requested_mode == LinkageMode::kLinkage && input.left_switch == SwitchPosition::kUnknown)) {
    make_disabled_output(input);
    manual_source_ = ManualInputSource::kNone;
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
    manual_source_ = ManualInputSource::kNone;
    output_.manual_input_source = ManualInputSource::kNone;
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
         std::isfinite(input.yaw_rate_rad_s) &&
         std::isfinite(input.motor_a_angle_rad) && std::isfinite(input.motor_b_angle_rad);
}

bool LinkageController::config_valid() const
{
  return std::isfinite(config_.motor_a_to_yaw_ratio) && std::isfinite(config_.motor_b_ratio_down) &&
         std::isfinite(config_.motor_b_ratio_middle) && std::isfinite(config_.motor_b_ratio_up) &&
         std::isfinite(config_.motor_b_reset_to_yaw_ratio) &&
         std::isfinite(config_.manual_detection_threshold_rad) &&
         std::isfinite(config_.target_reached_threshold_rad) &&
         std::isfinite(config_.yaw_motion_threshold_rad_s) &&
         config_.manual_detection_threshold_rad > 0.0F &&
         config_.target_reached_threshold_rad > 0.0F && config_.yaw_motion_threshold_rad_s >= 0.0F &&
         absolute_value(config_.motor_a_to_yaw_ratio) > kMinimumNonzeroRatio &&
         absolute_value(config_.motor_b_reset_to_yaw_ratio) > kMinimumNonzeroRatio;
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

void LinkageController::capture_reset_reference(const LinkageInput & input)
{
  reset_reference_yaw_angle_rad_ = yaw_unwrapped_angle_rad_;
  reset_reference_motor_a_angle_rad_ = input.motor_a_angle_rad;
  reset_reference_motor_b_angle_rad_ = input.motor_b_angle_rad;
  reset_reference_calibrated_ = true;
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
  output_.manual_input_source = ManualInputSource::kNone;

  // Keeping targets at measured positions prevents a later motor layer from
  // creating a position jump if it temporarily ignores motor_enable.
  output_.motor_a_target_angle_rad = input.motor_a_angle_rad;
  output_.motor_b_target_angle_rad = input.motor_b_angle_rad;
  output_.reset_reference_calibrated = reset_reference_calibrated_;
}

void LinkageController::make_reset_output(const LinkageInput & input)
{
  output_.mode = LinkageMode::kReset;
  output_.manual_input_source = ManualInputSource::kNone;
  // Return motor R marks to the startup alignment, adjusted by C-board yaw.
  // BMI088 has no absolute heading sensor, so startup requires physical mark
  // alignment with right switch down; otherwise reset stays safely disabled.
  output_.motor_enable = reset_reference_calibrated_;
  output_.motor_b_ratio = 0.0F;
  const float yaw_from_reset_rad = yaw_unwrapped_angle_rad_ - reset_reference_yaw_angle_rad_;
  output_.motor_a_target_angle_rad = reset_reference_motor_a_angle_rad_ +
    yaw_from_reset_rad * config_.motor_a_to_yaw_ratio;
  output_.motor_b_target_angle_rad = reset_reference_motor_b_angle_rad_ +
    yaw_from_reset_rad * config_.motor_b_reset_to_yaw_ratio;
  output_.reset_reference_calibrated = reset_reference_calibrated_;
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

  float yaw_offset_rad = yaw_unwrapped_angle_rad_ - output_.yaw_reference_angle_rad;

  // Changing the left switch changes the B ratio. Re-anchor B at its current
  // measured position so selecting another ratio never creates a target jump.
  if (
    !motor_b_ratio_initialized_ ||
    absolute_value(motor_b_ratio - previous_motor_b_ratio_) > kMinimumNonzeroRatio) {
    output_.motor_b_reference_angle_rad =
      input.motor_b_angle_rad - yaw_offset_rad * motor_b_ratio;
    motor_b_ratio_initialized_ = true;
    previous_motor_b_ratio_ = motor_b_ratio;
    manual_source_ = ManualInputSource::kNone;
    output_.manual_input_source = ManualInputSource::kNone;
    manual_source_stationary_cycles_ = 0U;
  }

  const bool yaw_is_quiet = absolute_value(input.yaw_rate_rad_s) <= config_.yaw_motion_threshold_rad_s;
  if (!yaw_is_quiet) {
    manual_source_ = ManualInputSource::kNone;
    output_.manual_input_source = ManualInputSource::kNone;
    manual_source_stationary_cycles_ = 0U;
  }

  // While yaw is stationary, identify a motor whose position error is growing.
  // This distinguishes a hand-driven motor moving away from its target from a
  // PID-controlled follower that is moving toward its target. Do not require
  // both motors to be fully settled: real PID tracking error otherwise blocks
  // manual input completely.
  if (yaw_is_quiet) {
    const float motor_a_error_rad = input.motor_a_angle_rad - previous_motor_a_target_angle_rad_;
    const float motor_b_error_rad = input.motor_b_angle_rad - previous_motor_b_target_angle_rad_;
    const float motor_a_previous_error_rad =
      previous_motor_a_angle_rad_ - previous_motor_a_target_angle_rad_;
    const float motor_b_previous_error_rad =
      previous_motor_b_angle_rad_ - previous_motor_b_target_angle_rad_;
    const float motor_a_error_growth_rad =
      absolute_value(motor_a_error_rad) - absolute_value(motor_a_previous_error_rad);
    const float motor_b_error_growth_rad =
      absolute_value(motor_b_error_rad) - absolute_value(motor_b_previous_error_rad);
    const float minimum_error_growth_rad = kEncoderResolutionRad * 0.5F;

    if (manual_source_ == ManualInputSource::kNone) {
      const bool motor_a_manual_candidate =
        absolute_value(motor_a_error_rad) > config_.manual_detection_threshold_rad &&
        motor_a_error_growth_rad > minimum_error_growth_rad;
      const bool motor_b_manual_candidate =
        absolute_value(motor_b_error_rad) > config_.manual_detection_threshold_rad &&
        motor_b_error_growth_rad > minimum_error_growth_rad;

      if (motor_a_manual_candidate || motor_b_manual_candidate) {
        manual_source_ = motor_a_manual_candidate && motor_b_manual_candidate ?
          (motor_a_error_growth_rad >= motor_b_error_growth_rad ?
            ManualInputSource::kMotorA : ManualInputSource::kMotorB) :
          (motor_a_manual_candidate ? ManualInputSource::kMotorA : ManualInputSource::kMotorB);
        // Anchor the virtual-zero shift to the old target. Applying the full
        // accumulated error makes the commanded source motor catch its hand-
        // moved position immediately, while the other motor follows by ratio.
        manual_source_origin_angle_rad_ = manual_source_ == ManualInputSource::kMotorA ?
          previous_motor_a_target_angle_rad_ : previous_motor_b_target_angle_rad_;
        manual_source_origin_yaw_reference_rad_ = output_.yaw_reference_angle_rad;
        manual_source_stationary_cycles_ = 0U;
      }
    }

    if (manual_source_ == ManualInputSource::kMotorA) {
      output_.yaw_reference_angle_rad = manual_source_origin_yaw_reference_rad_ -
        (input.motor_a_angle_rad - manual_source_origin_angle_rad_) / config_.motor_a_to_yaw_ratio;
      if (absolute_value(input.motor_a_angle_rad - previous_motor_a_angle_rad_) >
          kEncoderResolutionRad * 0.5F) {
        manual_source_stationary_cycles_ = 0U;
      } else if (manual_source_stationary_cycles_ < kManualSourceQuietCycles) {
        ++manual_source_stationary_cycles_;
      }
    } else if (manual_source_ == ManualInputSource::kMotorB &&
               absolute_value(motor_b_ratio) > kMinimumNonzeroRatio) {
      output_.yaw_reference_angle_rad = manual_source_origin_yaw_reference_rad_ -
        (input.motor_b_angle_rad - manual_source_origin_angle_rad_) / motor_b_ratio;
      if (absolute_value(input.motor_b_angle_rad - previous_motor_b_angle_rad_) >
          kEncoderResolutionRad * 0.5F) {
        manual_source_stationary_cycles_ = 0U;
      } else if (manual_source_stationary_cycles_ < kManualSourceQuietCycles) {
        ++manual_source_stationary_cycles_;
      }
    }

    // Keep the source latched while it is moving so slow hand rotation is
    // accumulated. Release after 50 quiet 1 ms control cycles; a later input
    // can then select either motor as the new source.
    if (manual_source_stationary_cycles_ >= kManualSourceQuietCycles) {
      manual_source_ = ManualInputSource::kNone;
    }
  }

  yaw_offset_rad = yaw_unwrapped_angle_rad_ - output_.yaw_reference_angle_rad;
  output_.mode = LinkageMode::kLinkage;
  output_.motor_enable = true;
  output_.reset_active = false;
  output_.motor_b_ratio = motor_b_ratio;
  output_.motor_a_target_angle_rad =
    output_.motor_a_reference_angle_rad + yaw_offset_rad * config_.motor_a_to_yaw_ratio;
  output_.motor_b_target_angle_rad =
    output_.motor_b_reference_angle_rad + yaw_offset_rad * motor_b_ratio;
  output_.reset_reference_calibrated = reset_reference_calibrated_;
  output_.manual_input_source = manual_source_;
  previous_motor_a_target_angle_rad_ = output_.motor_a_target_angle_rad;
  previous_motor_b_target_angle_rad_ = output_.motor_b_target_angle_rad;
}

}  // namespace cboard
