# This message expresses DawnIK Solver goal

uint32 MODE_0=0   # IDLE
uint32 MODE_1=1   # POSITION GOAL
uint32 MODE_2=2   # ORIENTATION GOAL
uint32 MODE_3=4   # LOOK-AT GOAL
uint32 MODE_4=8
uint32 MODE_5=16
uint32 MODE_6=32
uint32 MODE_7=64
uint32 MODE_8=128
uint32 MODE_9=256

int64 mode

###################################################################################################
# MODE_0: IDLE
###################################################################################################

# Stops the optimization and generating output.

###################################################################################################
# MODE_1: POSITION GOAL
###################################################################################################

float64 m1_x
float64 m1_y
float64 m1_z
float64 m1_weight
float64 m1_limit_dist # TODO: handle zero cases

###################################################################################################
# MODE_2: ORIENTATION GOAL
###################################################################################################

float64 m2_w
float64 m2_x
float64 m2_y
float64 m2_z
float64 m2_weight

###################################################################################################
# MODE_3: LOOK AT GOAL
###################################################################################################

float64 m3_x
float64 m3_y
float64 m3_z
float64 m3_weight