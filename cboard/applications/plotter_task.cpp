#include <cstdint>

#include "cmsis_os.h"

#include "imu/bmi088.hpp"
#include "usart.h"

namespace
{
constexpr std::uint32_t kPeriodMs = 10U;  // 100 Hz
constexpr std::uint32_t kStatusEveryCycles = 100U;  // 1 s while the IMU is not ready

// Worst case: 6 values * 15 chars + 5 commas + newline = 96 bytes.
std::uint8_t tx_buffer[112];

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
      *out++ = '\r';
      *out++ = '\n';
    }
    else if (cycle % kStatusEveryCycles == 0U) {
      out = append_text(out, "IMU_WAIT,err=");
      out = append_uint(out, cboard::bmi088.last_error);
      *out++ = '\r';
      *out++ = '\n';
    }
    else {
      continue;
    }

    (void)HAL_UART_Transmit_DMA(&huart1, tx_buffer, static_cast<std::uint16_t>(out - begin));
  }
}
