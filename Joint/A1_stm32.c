#include "A1_stm32.h"
#include "hardware_config.h"
#include "control_logic.h"
#include <math.h>
#include <string.h>
#include "joint_control.h"
#include "attitude.h"

#define abs(x) ((x>=0)?(x):(-x))