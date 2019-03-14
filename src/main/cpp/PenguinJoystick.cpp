#include "PenguinJoystick.h"

PenguinJoystick::PenguinJoystick() {}

PenguinJoystick::~PenguinJoystick() { // Delete all pointers in button array
	if( m_Joystick != NULL) {
		for (int i = 0; i < maxButtons; i++) {
			if( buttonArray[i] != NULL ) {
				delete buttonArray[i];
				buttonArray[i] = NULL;
			}
		}
		m_Joystick = NULL;
	}
}

void PenguinJoystick::Init(frc::Joystick* theJoystick) { // Create list of buttons from referenced Joystick
	m_Joystick = theJoystick;

	for (int i = 0; i < maxButtons; i++) {
		buttonArray[i] = new frc::JoystickButton(m_Joystick, i + 1);
	}
}

void PenguinJoystick::ReadJoystick() { // Check all buttons to update values
	for (int i = 0; i < maxButtons; i++) {
		buttonValueArray[i] = buttonArray[i]->Get();
	}
}

bool PenguinJoystick::ReadButton(int buttonNumber) { // Retrieve specific button value from value array
	return buttonValueArray[buttonNumber - 1];
}

float PenguinJoystick::CheckLeftStickX() { // Get Joystick X-axis (left X on gamepad)
	return m_Joystick->GetX();
}

float PenguinJoystick::CheckLeftStickY() { // Get Joystick Y-axis (left Y on gamepad)
	return m_Joystick->GetY()*-1;
}

float PenguinJoystick::CheckRightStickX() { // Get Joystick Z-axis (right X on gamepad)
	return m_Joystick->GetZ();
}

float PenguinJoystick::CheckRightStickY() { // Get Joystick throttle (right Y on gamepad)
	return m_Joystick->GetThrottle();
}

int PenguinJoystick::GetPOV() { // Get d-pad direction
	return m_Joystick->GetPOV();
}




