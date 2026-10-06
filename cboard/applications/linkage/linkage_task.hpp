#ifndef CBOARD_APPLICATIONS_LINKAGE_LINKAGE_TASK_HPP_
#define CBOARD_APPLICATIONS_LINKAGE_LINKAGE_TASK_HPP_

#include "linkage_controller.hpp"

namespace cboard
{

extern LinkageOutput linkage_output;

}  // namespace cboard

extern "C" void linkage_task(void * argument);

#endif  // CBOARD_APPLICATIONS_LINKAGE_LINKAGE_TASK_HPP_
