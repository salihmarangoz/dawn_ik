#ifndef __ROBOT_CONFIGURATION_MAIN_HEADER__
#define __ROBOT_CONFIGURATION_MAIN_HEADER__

// TODO: this section needs to be dynamically configured while compiling to support multiple robots

#if USE_HORTI
#include <dawn_ik/robot_configuration/horti.h>
#elif USE_LITE6
#include <dawn_ik/robot_configuration/lite6.h>
#elif USE_MICK
#include <dawn_ik/robot_configuration/mick.h>
#elif USE_MICK_OTHER
#include <dawn_ik/robot_configuration/mick_other.h>
#elif USE_TROLLEY
#include <dawn_ik/robot_configuration/trolley.h>
#else
#include <dawn_ik/robot_configuration/autogen_test.h>
#endif


#endif // __ROBOT_CONFIGURATION_MAIN_HEADER__