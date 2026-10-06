#include "motor_state.hpp"

#include "stm32f4xx_hal.h"

namespace cboard
{

Gm6020State gm6020_state;

int Gm6020State::slot_from_can_id(std::uint16_t can_id)
{
  if (can_id < 0x205U || can_id > 0x208U) {
    return -1;
  }
  return static_cast<int>(can_id - 0x205U);
}

bool Gm6020State::update(
  std::uint16_t can_id, const std::uint8_t data[8], std::uint32_t now_tick)
{
  const int slot = slot_from_can_id(can_id);
  if (slot < 0 || !decode_gm6020_feedback(can_id, data, slots_[slot].feedback)) {
    return false;
  }

  slots_[slot].angle_tracker.update(slots_[slot].feedback.encoder);
  slots_[slot].last_update_tick = now_tick;
  slots_[slot].valid = true;
  return true;
}

MotorMeasurement Gm6020State::measurement(
  std::uint16_t can_id, std::uint32_t now_tick) const
{
  MotorMeasurement result;
  const int slot = slot_from_can_id(can_id);
  if (slot < 0) {
    return result;
  }

  const Slot & state = slots_[slot];
  result.angle_rad = state.angle_tracker.angle_rad();
  result.speed_rpm = state.feedback.speed_rpm;
  result.current = state.feedback.current;
  result.temperature_c = state.feedback.temperature_c;
  result.last_update_tick = state.last_update_tick;
  result.valid = state.valid && (now_tick - state.last_update_tick <= kFeedbackTimeoutMs);
  return result;
}

}  // namespace cboard

/* C-linkage entry point used by the CAN ISR layer. It only updates the
 * feedback cache; no motor command is generated from an interrupt. */
extern "C" void cboard_motor_can_on_feedback(
  std::uint16_t standard_id, const std::uint8_t * data, std::uint8_t length)
{
  if (data == nullptr || length < 7U) {
    return;
  }
  (void)cboard::gm6020_state.update(standard_id, data, HAL_GetTick());
}
