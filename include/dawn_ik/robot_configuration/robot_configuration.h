#ifndef __ROBOT_CONFIGURATION_MAIN_HEADER__
#define __ROBOT_CONFIGURATION_MAIN_HEADER__

// TODO: this section needs to be dynamically configured while compiling to support multiple robots

#if USE_MICK_OTHER
#include <dawn_ik/robot_configuration/mick_other.h>
#elif USE_REPAIR_ARM_1
#include <dawn_ik/robot_configuration/repair_arm_1.h>
#elif USE_REPAIR_ARM_2
#include <dawn_ik/robot_configuration/repair_arm_2.h>
#else
#include <dawn_ik/robot_configuration/autogen_test.h>
#endif


#endif // __ROBOT_CONFIGURATION_MAIN_HEADER__