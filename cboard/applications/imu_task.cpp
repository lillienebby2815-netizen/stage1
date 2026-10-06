#include "cmsis_os.h"

#include "imu/bmi088.hpp"
#include "imu/yaw_estimator.hpp"

namespace
{
// This is the course3 C-board mounting transform: sensor X/Y are rotated into
// the robot frame while sensor Z keeps its direction.
constexpr float R_AB[3][3] = {
  {0.0F, -1.0F, 0.0F},
  {1.0F, 0.0F, 0.0F},
  {0.0F, 0.0F, 1.0F}};
}

namespace cboard
{
BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, R_AB);
}

extern "C" void imu_task_entry(void * argument)
{
  (void)argument;

  // Keep retrying until both BMI088 dies answer with the expected chip IDs.
  // This prevents one transient power-up failure from permanently disabling
  // the task; the plotter continues sending zeroes while initialization waits.
  while (!cboard::bmi088.init()) {
    osDelay(100U);
  }

  for (;;) {
    if (cboard::bmi088.update()) {
      cboard::yaw_estimator.update(
        cboard::bmi088.gyro[2], osKernelGetTickCount(), osKernelGetTickFreq());
    }
    else {
      cboard::yaw_estimator.invalidate();
    }
    osDelay(1U);
  }
}
