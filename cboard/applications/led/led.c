#include "led.h"

#include <stdint.h>

#include "gpio.h"

enum
{
  kLedCount = 3U,
  kLedOffLevel = GPIO_PIN_SET,
  kLedOnLevel = GPIO_PIN_RESET,
};

void led_show_chase_step(void)
{
  static const uint16_t led_pins[kLedCount] = {
    LED_B_Pin,
    LED_G_Pin,
    LED_R_Pin,
  };
  static uint32_t current_led = 0U;

  HAL_GPIO_WritePin(GPIOH, LED_R_Pin | LED_G_Pin | LED_B_Pin, kLedOffLevel);
  HAL_GPIO_WritePin(GPIOH, led_pins[current_led], kLedOnLevel);
  current_led = (current_led + 1U) % kLedCount;
}
