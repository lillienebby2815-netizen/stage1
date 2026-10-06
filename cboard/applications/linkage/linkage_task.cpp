#include "linkage_task.hpp"

#include "cmsis_os.h"

#include "imu/yaw_estimator.hpp"
#include "motor/motor_state.hpp"
#include "remote/dbus.hpp"

namespace
{
// These IDs are placeholders until the lab wiring and motor labels are
// confirmed. Invalid/no feedback keeps the controller in disabled mode.
constexpr std::uint16_t kMotorAFeedbackId = 0x205U;
constexpr std::uint16_t kMotorBFeedbackId = 0x206U;
}

namespace cboard
{

LinkageOutput linkage_output;

namespace
{
LinkageController controller;
}

}  // namespace cboard

extern "C" void linkage_task(void * argument)
{
  (void)argument;

  for (;;) {
    const std::uint32_t now_tick = osKernelGetTickCount();
    const cboard::Dt7Snapshot remote = cboard::dt7_receiver.snapshot(now_tick);
    const cboard::MotorMeasurement motor_a =
      cboard::gm6020_state.measurement(kMotorAFeedbackId, now_tick);
    const cboard::MotorMeasurement motor_b =
      cboard::gm6020_state.measurement(kMotorBFeedbackId, now_tick);

    cboard::LinkageInput input;
    input.yaw_angle_rad = cboard::yaw_estimator.angle_rad();
    input.motor_a_angle_rad = motor_a.angle_rad;
    input.motor_b_angle_rad = motor_b.angle_rad;
    input.left_switch = remote.left_switch;
    input.right_switch = remote.right_switch;
    input.remote_valid = remote.valid;
    input.imu_valid = cboard::yaw_estimator.valid();
    input.motor_a_valid = motor_a.valid;
    input.motor_b_valid = motor_b.valid;

    // This only computes a safe target. A future CAN transmit task must check
    // motor_enable and translate the target into a bounded current command.
    cboard::linkage_output = cboard::controller.update(input);
    osDelay(1U);
  }
}
