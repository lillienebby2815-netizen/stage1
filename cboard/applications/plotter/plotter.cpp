#include "plotter.hpp"

namespace cboard
{

Plotter::Plotter(UART_HandleTypeDef * huart, bool use_dma)
: huart_(huart), use_dma_(use_dma), hal_status_(HAL_OK), frame_{}
{
}

void Plotter::plot(float value1)
{
  frame_.size = static_cast<std::uint8_t>(sizeof(float));
  frame_.data[0] = value1;
  send();
}

void Plotter::plot(float value1, float value2)
{
  frame_.size = static_cast<std::uint8_t>(2U * sizeof(float));
  frame_.data[0] = value1;
  frame_.data[1] = value2;
  send();
}

void Plotter::plot(float value1, float value2, float value3)
{
  frame_.size = static_cast<std::uint8_t>(3U * sizeof(float));
  frame_.data[0] = value1;
  frame_.data[1] = value2;
  frame_.data[2] = value3;
  send();
}

void Plotter::send()
{
  if (huart_->gState != HAL_UART_STATE_READY) {
    return;
  }

  const std::uint16_t frame_length = static_cast<std::uint16_t>(
    sizeof(frame_.start) + sizeof(frame_.size) + frame_.size);

  if (use_dma_) {
    hal_status_ = HAL_UART_Transmit_DMA(
      huart_, reinterpret_cast<std::uint8_t *>(&frame_), frame_length);
  }
  else {
    hal_status_ = HAL_UART_Transmit(
      huart_, reinterpret_cast<std::uint8_t *>(&frame_), frame_length, 0xFFU);
  }
}

}  // namespace cboard
