#ifndef CBOARD_APPLICATIONS_REMOTE_DBUS_HPP_
#define CBOARD_APPLICATIONS_REMOTE_DBUS_HPP_

#include <cstdint>

#include "linkage/linkage_controller.hpp"

namespace cboard
{

struct Dt7Snapshot
{
  std::int16_t channel[4] = {0, 0, 0, 0};
  SwitchPosition left_switch = SwitchPosition::kUnknown;
  SwitchPosition right_switch = SwitchPosition::kUnknown;
  bool valid = false;
  std::uint32_t last_frame_tick = 0U;
};

// DBUS receiver in the style of sp_middleware: USART3 receives to the IDLE
// line, so every event is one whole 18-byte frame and no byte sliding is needed.
class Dt7Receiver
{
public:
  static constexpr std::uint32_t kFrameTimeoutMs = 100U;
  static constexpr std::uint8_t kFrameLength = 18U;

  // Returns the buffer handed to HAL_UARTEx_ReceiveToIdle_IT.
  std::uint8_t * rx_buffer() { return buffer_; }
  // Called from the RX event callback with the number of bytes received.
  void on_rx_event(std::uint16_t size, std::uint32_t now_tick);

  Dt7Snapshot snapshot(std::uint32_t now_tick) const;

  std::uint32_t event_count() const { return event_count_; }
  std::uint32_t frame_count() const { return frame_count_; }
  std::uint16_t last_event_size() const { return last_size_; }
  // Copies the last 18-byte frame that passed the size check.
  void copy_last_frame(std::uint8_t (&out)[kFrameLength]) const;

private:
  static constexpr std::uint16_t kChannelMin = 364U;
  static constexpr std::uint16_t kChannelMax = 1684U;

  std::uint8_t buffer_[kFrameLength] = {};
  volatile std::uint8_t last_frame_[kFrameLength] = {};
  volatile std::uint16_t last_size_ = 0U;
  volatile std::uint32_t event_count_ = 0U;
  volatile std::uint32_t frame_count_ = 0U;
  volatile std::int16_t channels_[4] = {0, 0, 0, 0};
  volatile SwitchPosition left_switch_ = SwitchPosition::kUnknown;
  volatile SwitchPosition right_switch_ = SwitchPosition::kUnknown;
  volatile bool frame_received_ = false;
  volatile std::uint32_t last_frame_tick_ = 0U;
  volatile std::uint32_t frame_sequence_ = 0U;

  static SwitchPosition decode_switch(std::uint8_t value);
};

extern Dt7Receiver dt7_receiver;

}  // namespace cboard

extern "C" void cboard_dt7_start_receive(void);

#endif  // CBOARD_APPLICATIONS_REMOTE_DBUS_HPP_
