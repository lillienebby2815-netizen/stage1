#ifndef CBOARD_APPLICATIONS_MOTOR_GM6020_PROTOCOL_HPP_
#define CBOARD_APPLICATIONS_MOTOR_GM6020_PROTOCOL_HPP_

#include <cstdint>

namespace cboard
{

struct Gm6020Feedback
{
  std::uint16_t encoder = 0U;
  std::int16_t speed_rpm = 0;
  std::int16_t current = 0;
  std::uint8_t temperature_c = 0U;
};

struct Gm6020AngleTracker
{
  bool initialized = false;
  std::int32_t unwrapped_encoder = 0;
  std::uint16_t previous_encoder = 0U;

  void reset();
  float update(std::uint16_t encoder);
  float angle_rad() const;
};

struct Gm6020CommandFrame
{
  std::uint16_t can_id = 0x1FFU;
  std::int16_t current[4] = {0, 0, 0, 0};
  std::uint8_t data[8] = {};
};

bool decode_gm6020_feedback(
  std::uint16_t can_id, const std::uint8_t data[8], Gm6020Feedback & feedback);

Gm6020CommandFrame make_gm6020_command(
  std::uint16_t can_id, const std::int16_t current[4]);

}  // namespace cboard

#endif  // CBOARD_APPLICATIONS_MOTOR_GM6020_PROTOCOL_HPP_
