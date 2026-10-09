#ifndef CBOARD_APPLICATIONS_IMU_YAW_ESTIMATOR_HPP_
#define CBOARD_APPLICATIONS_IMU_YAW_ESTIMATOR_HPP_

#include <cstdint>

namespace cboard
{

class YawEstimator
{
public:
  void reset();
  void invalidate();
  void update(float gyro_z_rad_s, std::uint32_t now_tick, std::uint32_t tick_frequency_hz);

  float angle_rad() const { return yaw_angle_rad_; }
  float yaw_rate_rad_s() const { return gyro_z_rate_rad_s_; }
  bool valid() const { return valid_; }
  bool calibrating() const { return !valid_; }

private:
  static constexpr std::uint32_t kCalibrationDurationMs = 500U;

  volatile float yaw_angle_rad_ = 0.0F;
  volatile float gyro_z_rate_rad_s_ = 0.0F;
  volatile float gyro_bias_rad_s_ = 0.0F;
  volatile float bias_sum_rad_s_ = 0.0F;
  volatile std::uint32_t bias_samples_ = 0U;
  volatile std::uint32_t calibration_elapsed_ms_ = 0U;
  volatile std::uint32_t previous_tick_ = 0U;
  volatile bool has_previous_tick_ = false;
  volatile bool valid_ = false;
};

extern YawEstimator yaw_estimator;

}  // namespace cboard

#endif  // CBOARD_APPLICATIONS_IMU_YAW_ESTIMATOR_HPP_
