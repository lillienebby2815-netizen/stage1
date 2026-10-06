#ifndef CBOARD_APPLICATIONS_PLOTTER_PLOTTER_HPP_
#define CBOARD_APPLICATIONS_PLOTTER_PLOTTER_HPP_

#include <cstdint>

#include "usart.h"

namespace cboard
{

constexpr std::size_t PLOTTER_FLOAT_NUM = 3U;

struct __attribute__((packed)) PlotFrame
{
  std::uint8_t start[2] = {0xAAU, 0xBBU};
  std::uint8_t size = 0U;
  float data[PLOTTER_FLOAT_NUM] = {};
};

class Plotter
{
public:
  explicit Plotter(UART_HandleTypeDef * huart, bool use_dma = true);

  void plot(float value1);
  void plot(float value1, float value2);
  void plot(float value1, float value2, float value3);

private:
  UART_HandleTypeDef * huart_;
  bool use_dma_;
  HAL_StatusTypeDef hal_status_;
  PlotFrame frame_;

  void send();
};

}  // namespace cboard

#endif  // CBOARD_APPLICATIONS_PLOTTER_PLOTTER_HPP_
