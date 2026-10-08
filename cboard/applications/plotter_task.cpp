#include <cstdint>

#include "cmsis_os.h"

#include "imu/bmi088.hpp"
#include "linkage/linkage_task.hpp"
#include "remote/dbus.hpp"
#include "usart.h"

namespace
{
constexpr std::uint32_t kPeriodMs = 10U;  // 100 Hz
constexpr std::uint32_t kStatusEveryCycles = 100U;  // 1 s while the IMU is not ready
constexpr std::uint32_t kDt7EveryCycles = 10U;  // 10 Hz DT7 debug line

// IMU line: worst case 6 values * 15 chars + 5 commas + CRLF = 97 bytes.
// DT7 line: "DT7," + 7 small ints + 3 counters + commas + CRLF < 100 bytes.
// RAW line: "RAW," + 64 hex chars + CRLF = 70 bytes.
// CHG line: "CHG," + 50 hex chars + CRLF = 56 bytes.
std::uint8_t tx_buffer[512];

char * append_uint(char * out, std::uint32_t value)
{
  char digits[10];
  std::uint8_t count = 0U;
  do {
    digits[count++] = static_cast<char>('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U);
  while (count > 0U) {
    *out++ = digits[--count];
  }
  return out;
}

char * append_int(char * out, std::int32_t value)
{
  if (value < 0) {
    *out++ = '-';
    return append_uint(out, static_cast<std::uint32_t>(-(value + 1)) + 1U);
  }
  return append_uint(out, static_cast<std::uint32_t>(value));
}

// Fixed point with three decimals. newlib-nano printf has no float support,
// so format by hand instead of pulling in -u _printf_float.
char * append_fixed3(char * out, float value)
{
  float scaled = value * 1000.0F;
  if (!(scaled < 2.0e9F && scaled > -2.0e9F)) {  // also rejects NaN
    scaled = 0.0F;
  }
  const std::int32_t milli =
    static_cast<std::int32_t>(scaled >= 0.0F ? scaled + 0.5F : scaled - 0.5F);
  const std::uint32_t magnitude = static_cast<std::uint32_t>(milli < 0 ? -milli : milli);

  if (milli < 0) {
    *out++ = '-';
  }
  out = append_uint(out, magnitude / 1000U);
  *out++ = '.';
  const std::uint32_t fraction = magnitude % 1000U;
  *out++ = static_cast<char>('0' + fraction / 100U);
  *out++ = static_cast<char>('0' + (fraction / 10U) % 10U);
  *out++ = static_cast<char>('0' + fraction % 10U);
  return out;
}

char * append_text(char * out, const char * text)
{
  while (*text != '\0') {
    *out++ = *text++;
  }
  return out;
}

char * append_crlf(char * out)
{
  *out++ = '\r';
  *out++ = '\n';
  return out;
}
}  // namespace

extern "C" void plotter_task(void * argument)
{
  (void)argument;

  std::uint32_t cycle = 0U;
  for (;;) {
    osDelay(kPeriodMs);
    ++cycle;

    // Only touch the buffer once the previous DMA transfer has finished.
    if (huart1.gState != HAL_UART_STATE_READY) {
      continue;
    }

    char * const begin = reinterpret_cast<char *>(tx_buffer);
    char * out = begin;

    if (cboard::bmi088.initialized) {
      // One line per sample for SerialPlot (ASCII, comma separated):
      // ax, ay, az [m/s^2], gx, gy, gz [rad/s]
      out = append_fixed3(out, cboard::bmi088.acc[0]);
      *out++ = ',';
      out = append_fixed3(out, cboard::bmi088.acc[1]);
      *out++ = ',';
      out = append_fixed3(out, cboard::bmi088.acc[2]);
      *out++ = ',';
      out = append_fixed3(out, cboard::bmi088.gyro[0]);
      *out++ = ',';
      out = append_fixed3(out, cboard::bmi088.gyro[1]);
      *out++ = ',';
      out = append_fixed3(out, cboard::bmi088.gyro[2]);
      out = append_crlf(out);
    }
    else if (cycle % kStatusEveryCycles == 0U) {
      out = append_text(out, "IMU_WAIT,err=");
      out = append_uint(out, cboard::bmi088.last_error);
      out = append_crlf(out);
    }

    // LNK,ratio,yaw,a_target,a_angle,b_target,b_angle,a_current,b_current
    // Angles in rad, currents in raw GM6020 units. Compare b_target with
    // b_angle to see tracking error and whether current stays small/saturated.
    out = append_text(out, "LNK,");
    out = append_fixed3(out, cboard::linkage_output.motor_b_ratio);
    *out++ = ',';
    out = append_fixed3(out, cboard::linkage_yaw_angle_rad);
    *out++ = ',';
    out = append_fixed3(out, cboard::linkage_output.motor_a_target_angle_rad);
    *out++ = ',';
    out = append_fixed3(out, cboard::linkage_motor_a_angle_rad);
    *out++ = ',';
    out = append_fixed3(out, cboard::linkage_output.motor_b_target_angle_rad);
    *out++ = ',';
    out = append_fixed3(out, cboard::linkage_motor_b_angle_rad);
    *out++ = ',';
    out = append_int(out, cboard::linkage_current_a_raw);
    *out++ = ',';
    out = append_int(out, cboard::linkage_current_b_raw);
    out = append_crlf(out);

    if (cycle % kDt7EveryCycles == 0U) {
      // DT7,ch0,ch1,ch2,ch3,left,right,valid,rx_bytes,frames,tail_byte
      // Channels are -660..660 around centre; switches are -1 down, 0 mid, 1 up, 2 unknown.
      const cboard::Dt7Snapshot remote = cboard::dt7_receiver.snapshot(HAL_GetTick());
      out = append_text(out, "DT7");
      for (const std::int16_t channel : remote.channel) {
        *out++ = ',';
        out = append_int(out, channel);
      }
      *out++ = ',';
      out = append_int(out, static_cast<std::int32_t>(remote.left_switch));
      *out++ = ',';
      out = append_int(out, static_cast<std::int32_t>(remote.right_switch));
      *out++ = ',';
      out = append_uint(out, remote.valid ? 1U : 0U);
      *out++ = ',';
      out = append_uint(out, cboard::dt7_receiver.event_count());
      *out++ = ',';
      out = append_uint(out, cboard::dt7_receiver.frame_count());
      *out++ = ',';
      out = append_uint(out, cboard::dt7_receiver.last_event_size());
      out = append_crlf(out);

      // RAW,<last accepted 18-byte frame in hex>
      std::uint8_t frame[cboard::Dt7Receiver::kFrameLength];
      cboard::dt7_receiver.copy_last_frame(frame);
      out = append_text(out, "RAW,");
      for (const std::uint8_t value : frame) {
        constexpr char kHex[] = "0123456789ABCDEF";
        *out++ = kHex[value >> 4U];
        *out++ = kHex[value & 0x0FU];
      }
      out = append_crlf(out);
    }

    if (out == begin) {
      continue;
    }

    (void)HAL_UART_Transmit_DMA(&huart1, tx_buffer, static_cast<std::uint16_t>(out - begin));
  }
}
