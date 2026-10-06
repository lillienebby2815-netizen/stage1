#include "gm6020_protocol.hpp"

#include <cmath>

namespace
{
constexpr std::uint16_t kFeedbackIdFirst = 0x205U;
constexpr std::uint16_t kFeedbackIdLast = 0x208U;
constexpr std::uint16_t kCommandIdFirstGroup = 0x1FFU;
constexpr std::uint16_t kCommandIdSecondGroup = 0x2FFU;
constexpr std::int32_t kEncoderCountsPerRevolution = 8192;
constexpr float kTwoPi = 6.28318530717958647692F;
}

namespace cboard
{

void Gm6020AngleTracker::reset()
{
  initialized = false;
  unwrapped_encoder = 0;
  previous_encoder = 0U;
}

float Gm6020AngleTracker::update(std::uint16_t encoder)
{
  if (!initialized) {
    previous_encoder = encoder;
    unwrapped_encoder = static_cast<std::int32_t>(encoder);
    initialized = true;
    return angle_rad();
  }

  std::int32_t delta = static_cast<std::int32_t>(encoder) -
    static_cast<std::int32_t>(previous_encoder);
  if (delta > (kEncoderCountsPerRevolution / 2)) {
    delta -= kEncoderCountsPerRevolution;
  }
  else if (delta < -(kEncoderCountsPerRevolution / 2)) {
    delta += kEncoderCountsPerRevolution;
  }

  unwrapped_encoder += delta;
  previous_encoder = encoder;
  return angle_rad();
}

float Gm6020AngleTracker::angle_rad() const
{
  return static_cast<float>(unwrapped_encoder) * kTwoPi /
         static_cast<float>(kEncoderCountsPerRevolution);
}

bool decode_gm6020_feedback(
  std::uint16_t can_id, const std::uint8_t data[8], Gm6020Feedback & feedback)
{
  if (data == nullptr || can_id < kFeedbackIdFirst || can_id > kFeedbackIdLast) {
    return false;
  }

  feedback.encoder = static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(data[0]) << 8U) | data[1]);
  feedback.speed_rpm = static_cast<std::int16_t>(
    (static_cast<std::uint16_t>(data[2]) << 8U) | data[3]);
  feedback.current = static_cast<std::int16_t>(
    (static_cast<std::uint16_t>(data[4]) << 8U) | data[5]);
  feedback.temperature_c = data[6];
  return true;
}

Gm6020CommandFrame make_gm6020_command(
  std::uint16_t can_id, const std::int16_t current[4])
{
  Gm6020CommandFrame frame;
  frame.can_id = can_id == kCommandIdSecondGroup ? kCommandIdSecondGroup :
    kCommandIdFirstGroup;
  if (current == nullptr) {
    return frame;
  }
  for (std::uint8_t index = 0U; index < 4U; ++index) {
    frame.current[index] = current[index];
    frame.data[index * 2U] = static_cast<std::uint8_t>((current[index] >> 8) & 0xFF);
    frame.data[index * 2U + 1U] = static_cast<std::uint8_t>(current[index] & 0xFF);
  }
  return frame;
}

}  // namespace cboard
