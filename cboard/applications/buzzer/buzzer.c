#include "buzzer.h"

#include <stddef.h>
#include <stdint.h>

#include "cmsis_os.h"
#include "tim.h"

enum
{
  kBuzzerTimerClockHz = 1000000U,
  kBuzzerNoteDurationMs = 80U,
  kBuzzerNoteGapMs = 20U,
};

void buzzer_play_startup_melody(void)
{
  static const uint16_t melody_hz[] = {
    330U, 330U, 349U, 392U, 392U, 349U, 330U, 294U,
  };

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  for (size_t note_index = 0U;
       note_index < (sizeof(melody_hz) / sizeof(melody_hz[0]));
       ++note_index)
  {
    const uint32_t period = kBuzzerTimerClockHz / melody_hz[note_index];
    __HAL_TIM_SET_AUTORELOAD(&htim4, period - 1U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period / 2U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    osDelay(kBuzzerNoteDurationMs);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    osDelay(kBuzzerNoteGapMs);
  }

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}
