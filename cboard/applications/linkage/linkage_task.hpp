#ifndef CBOARD_APPLICATIONS_LINKAGE_LINKAGE_TASK_HPP_
#define CBOARD_APPLICATIONS_LINKAGE_LINKAGE_TASK_HPP_

#include "linkage_controller.hpp"

namespace cboard
{

extern LinkageOutput linkage_output;
extern volatile std::int16_t linkage_current_a_raw;
extern volatile std::int16_t linkage_current_b_raw;
extern volatile float linkage_yaw_angle_rad;
extern volatile float linkage_motor_a_angle_rad;
extern volatile float linkage_motor_b_angle_rad;
extern volatile bool linkage_remote_valid;
extern volatile bool linkage_imu_valid;
extern volatile bool linkage_motor_a_valid;
extern volatile bool linkage_motor_b_valid;

}  // namespace cboard

extern "C" void linkage_task(void * argument);

#endif  // CBOARD_APPLICATIONS_LINKAGE_LINKAGE_TASK_HPP_
