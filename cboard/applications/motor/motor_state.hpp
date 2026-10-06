#ifndef CBOARD_APPLICATIONS_MOTOR_MOTOR_STATE_HPP_
#define CBOARD_APPLICATIONS_MOTOR_MOTOR_STATE_HPP_

#include <cstdint>

#include "gm6020_protocol.hpp"

namespace cboard
{

struct MotorMeasurement
{
  float angle_rad = 0.0F;
  std::int16_t speed_rpm = 0;
  std::int16_t current = 0;
  std::uint8_t temperature_c = 0U;
  bool valid = false;
  std::uint32_t last_update_tick = 0U;
};

class Gm6020State
{
public:
  static constexpr std::uint32_t kFeedbackTimeoutMs = 100U;

  bool update(
    std::uint16_t can_id, const std::uint8_t data[8], std::uint32_t now_tick);
  MotorMeasurement measurement(std::uint16_t can_id, std::uint32_t now_tick) const;

private:
  struct Slot
  {
    Gm6020AngleTracker angle_tracker;
    Gm6020Feedback feedback;
    std::uint32_t last_update_tick = 0U;
    bool valid = false;
  } slots_[4];

  static int slot_from_can_id(std::uint16_t can_id);
};

extern Gm6020State gm6020_state;

}  // namespace cboard

#endif  // CBOARD_APPLICATIONS_MOTOR_MOTOR_STATE_HPP_
