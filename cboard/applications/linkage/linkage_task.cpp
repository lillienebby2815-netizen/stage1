#include "linkage_task.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "can.h"
#include "cmsis_os.h"

#include "imu/yaw_estimator.hpp"
#include "motor/gm6020_protocol.hpp"
#include "motor/motor_state.hpp"
#include "remote/dbus.hpp"

namespace
{
// These IDs are placeholders until the lab wiring and motor labels are
// confirmed. Invalid/no feedback keeps the controller in disabled mode.
constexpr std::uint16_t kMotorAFeedbackId = 0x205U;
constexpr std::uint16_t kMotorBFeedbackId = 0x206U;

// GM6020 IDs 1..4 share the 0x1FE current-command frame.
constexpr std::uint16_t kGm6020CurrentCommandId = 0x1FEU;
// Enabled for the first controlled lab response test. Keep the right switch
// in the down position while powering the system; that path always commands
// zero current. Disable this again if the mechanical setup is not secured.
constexpr bool kMotorOutputEnabled = true;
// The linkage position loop is a current-command PID.  The integral term is
// deliberately bounded and reset whenever output is disabled, so a stalled
// motor cannot accumulate an unsafe command while the switch is down.
constexpr float kPositionKpRawPerRad = 1900.0F;
constexpr float kPositionKiRawPerRadSecond = 60.0F;
constexpr float kVelocityKdRawPerRadPerSecond = 45.0F;
// First controlled increase after the ±500 response test. Keep this below
// the GM6020 raw full scale and raise it only after checking mechanics and
// temperature at the lower setting.
constexpr std::int16_t kCurrentLimitRaw = 1500;
constexpr float kIntegralContributionLimitRaw = 100.0F;
constexpr float kTwoPi = 6.28318530717958647692F;

struct PositionPidState
{
  float integral_error_rad_s = 0.0F;
  std::uint32_t previous_tick = 0U;
  bool initialized = false;

  void reset()
  {
    integral_error_rad_s = 0.0F;
    previous_tick = 0U;
    initialized = false;
  }
};

std::int16_t clamp_current(float value)
{
  const float limited = std::max(
    -static_cast<float>(kCurrentLimitRaw),
    std::min(static_cast<float>(kCurrentLimitRaw), value));
  return static_cast<std::int16_t>(limited);
}

std::int16_t position_current(
  float target_angle_rad, const cboard::MotorMeasurement & measurement,
  PositionPidState & state, std::uint32_t now_tick)
{
  const float error_rad = target_angle_rad - measurement.angle_rad;
  const float speed_rad_per_second =
    static_cast<float>(measurement.speed_rpm) * kTwoPi / 60.0F;

  float dt_s = 0.0F;
  if (state.initialized) {
    const std::uint32_t tick_delta = now_tick - state.previous_tick;
    const std::uint32_t tick_frequency = osKernelGetTickFreq();
    // Ignore an abnormally long pause instead of integrating stale error.
    if (tick_frequency != 0U && tick_delta > 0U && tick_delta <= tick_frequency / 5U) {
      dt_s = static_cast<float>(tick_delta) / static_cast<float>(tick_frequency);
    }
  }
  state.previous_tick = now_tick;
  state.initialized = true;

  const float integral_error_limit =
    kIntegralContributionLimitRaw / kPositionKiRawPerRadSecond;
  const float proposed_integral = std::max(
    -integral_error_limit,
    std::min(integral_error_limit, state.integral_error_rad_s + error_rad * dt_s));
  const float proportional_raw = kPositionKpRawPerRad * error_rad;
  const float derivative_raw = -kVelocityKdRawPerRadPerSecond * speed_rad_per_second;
  const float proposed_raw =
    proportional_raw + kPositionKiRawPerRadSecond * proposed_integral + derivative_raw;

  // Anti-windup: if the output is already saturated and the error would push
  // it farther into saturation, hold the integrator. It can still unwind when
  // the error changes sign.
  const bool saturating_in_error_direction =
    (proposed_raw > static_cast<float>(kCurrentLimitRaw) && error_rad > 0.0F) ||
    (proposed_raw < -static_cast<float>(kCurrentLimitRaw) && error_rad < 0.0F);
  if (!saturating_in_error_direction) {
    state.integral_error_rad_s = proposed_integral;
  }

  const float current_raw =
    proportional_raw + kPositionKiRawPerRadSecond * state.integral_error_rad_s + derivative_raw;
  return clamp_current(current_raw);
}

void send_linkage_current_command(
  const cboard::LinkageOutput & output,
  const cboard::MotorMeasurement & motor_a,
  const cboard::MotorMeasurement & motor_b,
  std::uint32_t now_tick)
{
  static PositionPidState motor_a_pid;
  static PositionPidState motor_b_pid;
  std::int16_t current[4] = {0, 0, 0, 0};

  // The right-switch safety decision and feedback validity are checked again
  // here before any nonzero current is sent.
  if (
    kMotorOutputEnabled && output.motor_enable && motor_a.valid && motor_b.valid) {
    current[0] = position_current(
      output.motor_a_target_angle_rad, motor_a, motor_a_pid, now_tick);
    current[1] = position_current(
      output.motor_b_target_angle_rad, motor_b, motor_b_pid, now_tick);
  }
  else {
    motor_a_pid.reset();
    motor_b_pid.reset();
  }

  cboard::linkage_current_a_raw = current[0];
  cboard::linkage_current_b_raw = current[1];

  const cboard::Gm6020CommandFrame command =
    cboard::make_gm6020_command(kGm6020CurrentCommandId, current);
  (void)cboard_can_send_standard(command.can_id, command.data, 8U);
}
}

namespace cboard
{

LinkageOutput linkage_output;
volatile std::int16_t linkage_current_a_raw = 0;
volatile std::int16_t linkage_current_b_raw = 0;
volatile float linkage_yaw_angle_rad = 0.0F;
volatile float linkage_motor_a_angle_rad = 0.0F;
volatile float linkage_motor_b_angle_rad = 0.0F;
volatile bool linkage_remote_valid = false;
volatile bool linkage_imu_valid = false;
volatile bool linkage_motor_a_valid = false;
volatile bool linkage_motor_b_valid = false;

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

    cboard::linkage_yaw_angle_rad = input.yaw_angle_rad;
    cboard::linkage_motor_a_angle_rad = input.motor_a_angle_rad;
    cboard::linkage_motor_b_angle_rad = input.motor_b_angle_rad;
    cboard::linkage_remote_valid = input.remote_valid;
    cboard::linkage_imu_valid = input.imu_valid;
    cboard::linkage_motor_a_valid = input.motor_a_valid;
    cboard::linkage_motor_b_valid = input.motor_b_valid;

    cboard::linkage_output = cboard::controller.update(input);
    send_linkage_current_command(cboard::linkage_output, motor_a, motor_b, now_tick);
    osDelay(1U);
  }
}
