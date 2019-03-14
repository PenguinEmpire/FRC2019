#pragma once

// WARNING: experimental. don't use

#include "WPILib.h"
#include <math.h>

class PenguinUtil {
public:
    double calculateJoystickDampening(double rawAxisValue) {
        if(fabs(rawAxisValue) <= 0.3) {
            return 0.0;
        } else if(fabs(rawAxisValue) < 0.7) {
            return 1.0;
        } else {
            return 0.0;
        }
    }
}

