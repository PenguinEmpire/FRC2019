#pragma once

#include "WPILib.h"

const int maxButtons = 12;

class PenguinJoystick {
private:
    frc::Joystick* m_Joystick = NULL;
    frc::JoystickButton* buttonArray[maxButtons];
    bool buttonValueArray[maxButtons];

public:

    PenguinJoystick();
    virtual ~PenguinJoystick();
    void Init(frc::Joystick* theJoystick);

    void ReadJoystick();
    bool ReadButton(int buttonNumber);
    float CheckLeftStickX();
	float CheckLeftStickY();
	float CheckRightStickX();
	float CheckRightStickY();
	int GetPOV();
};