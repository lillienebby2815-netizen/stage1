#include "yaw_estimator.hpp"

#include <cmath>

namespace
{
constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;
constexpr float kStationaryGyroThresholdRadS = 0.15F;

float wrap_angle(float angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= kTwoPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += kTwoPi;
  }
  return angle_rad;
}
}

namespace cboard
{

YawEstimator yaw_estimator;

void YawEstimator::reset()
{
  yaw_angle_rad_ = 0.0F;
  gyro_z_rate_rad_s_ = 0.0F;
  gyro_bias_rad_s_ = 0.0F;
  bias_sum_rad_s_ = 0.0F;
  bias_samples_ = 0U;
  calibration_elapsed_ms_ = 0U;
  previous_tick_ = 0U;
  has_previous_tick_ = false;
  valid_ = false;
}

void YawEstimator::invalidate()
{
  reset();
}

void YawEstimator::update(
  float gyro_z_rad_s, std::uint32_t now_tick, std::uint32_t tick_frequency_hz)
{
  if (!std::isfinite(gyro_z_rad_s) || tick_frequency_hz == 0U) {
    yaw_angle_rad_ = 0.0F;
    gyro_z_rate_rad_s_ = 0.0F;
    gyro_bias_rad_s_ = 0.0F;
    bias_sum_rad_s_ = 0.0F;
    bias_samples_ = 0U;
    calibration_elapsed_ms_ = 0U;
    valid_ = false;
    has_previous_tick_ = false;
    return;
  }

  if (!has_previous_tick_) {
    previous_tick_ = now_tick;
    has_previous_tick_ = true;
    return;
  }

  const std::uint32_t tick_delta = now_tick - previous_tick_;
  previous_tick_ = now_tick;
  const std::uint32_t elapsed_ms =
    static_cast<std::uint32_t>((static_cast<std::uint64_t>(tick_delta) * 1000U) /
                               tick_frequency_hz);

  if (!valid_) {
    if (std::fabs(gyro_z_rad_s) > kStationaryGyroThresholdRadS) {
      bias_sum_rad_s_ = 0.0F;
      bias_samples_ = 0U;
      calibration_elapsed_ms_ = 0U;
      return;
    }

    bias_sum_rad_s_ += gyro_z_rad_s;
    ++bias_samples_;
    calibration_elapsed_ms_ += elapsed_ms;
    if (calibration_elapsed_ms_ >= kCalibrationDurationMs && bias_samples_ > 0U) {
      gyro_bias_rad_s_ = bias_sum_rad_s_ / static_cast<float>(bias_samples_);
      valid_ = true;
    }
    return;
  }

  // The linkage's manual-motor detector needs the stationary-corrected rate;
  // publishing the raw gyro bias here would make a steady C-board look active.
  gyro_z_rate_rad_s_ = gyro_z_rad_s - gyro_bias_rad_s_;

  // A long scheduler pause should not turn into a large artificial rotation.
  if (tick_delta == 0U || elapsed_ms > 100U) {
    return;
  }
  const float dt_s = static_cast<float>(tick_delta) / static_cast<float>(tick_frequency_hz);
  yaw_angle_rad_ = wrap_angle(yaw_angle_rad_ + (gyro_z_rad_s - gyro_bias_rad_s_) * dt_s);
}

}  // namespace cboard
