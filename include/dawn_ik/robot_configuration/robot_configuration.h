#ifndef __ROBOT_CONFIGURATION_MAIN_HEADER__
#define __ROBOT_CONFIGURATION_MAIN_HEADER__

// TODO: this section needs to be dynamically configured while compiling to support multiple robots

#if USE_MICK_OTHER
#include <dawn_ik/robot_configuration/mick_other.h>
#else
#include <dawn_ik/robot_configuration/autogen_test.h>
#endif


#endif // __ROBOT_CONFIGURATION_MAIN_HEADER__