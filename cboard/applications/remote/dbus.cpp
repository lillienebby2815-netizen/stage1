#include "dbus.hpp"

#include "stm32f4xx_hal.h"
#include "usart.h"

namespace cboard
{

Dt7Receiver dt7_receiver;

void Dt7Receiver::copy_last_frame(std::uint8_t (&out)[kFrameLength]) const
{
  for (std::uint8_t index = 0U; index < kFrameLength; ++index) {
    out[index] = last_frame_[index];
  }
}

Dt7Snapshot Dt7Receiver::snapshot(std::uint32_t now_tick) const
{
  Dt7Snapshot result;
  bool stable_snapshot = false;
  for (std::uint8_t attempt = 0U; attempt < 3U; ++attempt) {
    const std::uint32_t sequence_before = frame_sequence_;
    if ((sequence_before & 1U) != 0U) {
      continue;
    }

    result.channel[0] = channels_[0];
    result.channel[1] = channels_[1];
    result.channel[2] = channels_[2];
    result.channel[3] = channels_[3];
    result.left_switch = left_switch_;
    result.right_switch = right_switch_;
    result.last_frame_tick = last_frame_tick_;
    const bool frame_received = frame_received_;

    if (sequence_before == frame_sequence_) {
      stable_snapshot = true;
      result.valid = frame_received;
      break;
    }
  }

  const std::uint32_t age = now_tick - result.last_frame_tick;
  result.valid = stable_snapshot && result.valid && age <= kFrameTimeoutMs;
  return result;
}

SwitchPosition Dt7Receiver::decode_switch(std::uint8_t value)
{
  // DT7 switch values are 1=up, 3=middle, 2=down.
  switch (value) {
    case 1U:
      return SwitchPosition::kUp;
    case 2U:
      return SwitchPosition::kDown;
    case 3U:
      return SwitchPosition::kMiddle;
    default:
      return SwitchPosition::kUnknown;
  }
}

void Dt7Receiver::on_rx_event(std::uint16_t size, std::uint32_t now_tick)
{
  event_count_ = event_count_ + 1U;
  last_size_ = size;
  if (size != kFrameLength) {
    return;
  }

  const std::uint8_t * b = buffer_;
  for (std::uint8_t index = 0U; index < kFrameLength; ++index) {
    last_frame_[index] = b[index];
  }

  const std::uint16_t raw_channel[4] = {
    static_cast<std::uint16_t>((b[0] | (b[1] << 8U)) & 0x07FFU),
    static_cast<std::uint16_t>(((b[1] >> 3U) | (b[2] << 5U)) & 0x07FFU),
    static_cast<std::uint16_t>(((b[2] >> 6U) | (b[3] << 2U) | (b[4] << 10U)) & 0x07FFU),
    static_cast<std::uint16_t>(((b[4] >> 1U) | (b[5] << 7U)) & 0x07FFU)};
  const SwitchPosition left_switch = decode_switch((b[5] >> 6U) & 0x03U);
  const SwitchPosition right_switch = decode_switch((b[5] >> 4U) & 0x03U);

  for (const std::uint16_t channel : raw_channel) {
    if (channel < kChannelMin || channel > kChannelMax) {
      return;
    }
  }
  if (left_switch == SwitchPosition::kUnknown || right_switch == SwitchPosition::kUnknown) {
    return;
  }

  ++frame_sequence_;
  for (std::uint8_t index = 0U; index < 4U; ++index) {
    channels_[index] = static_cast<std::int16_t>(raw_channel[index]) - 1024;
  }
  left_switch_ = left_switch;
  right_switch_ = right_switch;
  last_frame_tick_ = now_tick;
  frame_received_ = true;
  frame_count_ = frame_count_ + 1U;
  ++frame_sequence_;
}

}  // namespace cboard

extern "C" void cboard_dt7_start_receive(void)
{
  if (huart3.RxState == HAL_UART_STATE_READY) {
    (void)HAL_UARTEx_ReceiveToIdle_IT(
      &huart3, cboard::dt7_receiver.rx_buffer(), cboard::Dt7Receiver::kFrameLength);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
  if (huart == &huart3) {
    cboard::dt7_receiver.on_rx_event(size, HAL_GetTick());
    cboard_dt7_start_receive();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    cboard_dt7_start_receive();
  }
}
