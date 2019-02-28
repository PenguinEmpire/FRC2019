#pragma once

// #include "Robot.h"
// #include <unorderedmap>
#include "frc/Timer.h"
#include "frc/DoubleSolenoid.h"

using std::unordered_map;

/* static constexpr */ std::unordered_map<frc::DoubleSolenoid::Value, frc::DoubleSolenoid::Value> reverseStates = {
  {frc::DoubleSolenoid::kReverse, frc::DoubleSolenoid::kForward}, 
  {frc::DoubleSolenoid::kForward, frc::DoubleSolenoid::kReverse}, 
};

class Pneumatic : public frc::DoubleSolenoid {
  public:

    frc::Timer* timer = new frc::Timer();
    frc::DoubleSolenoid::Value in;
    bool busy;

    double curTime;
    double goalTime;

    Pneumatic(int pcm, int pch_1, int pch_2, frc::DoubleSolenoid::Value state) 
      : frc::DoubleSolenoid(pcm, pch_1, pch_2)
    {
      Set(state);
      in = state;
      timer->Reset();

      // Toggle(); // testing
    }

    void Toggle() {
      if (!busy) {
        Set(reverseStates[Get()]);
      }
    }

    void Toggle(bool btn) {
      if (!busy && btn) {
        Toggle();
      }
    }

    void ToggleTimed(double seconds) {
      curTime = timer->Get();
      goalTime = curTime + seconds;
      printf("curTime: %f", curTime);
      Toggle();
      busy = true;
      // timer->Start();
    }

    void ToggleTimed(double seconds, bool btn) {
      if (btn && !busy) {
        ToggleTimed(seconds);
      }
    }

    void PenguinUpdate() {
      if (busy) {
        if (timer->Get() >= goalTime) {
          Toggle();
          busy = false;
          // timer->Stop();
        }
      }
    }
  
  private:
    // static constexpr std::unordered_map<frc::DoubleSolenoid::Value, frc::DoubleSolenoid::Value> reverseStates = {
    //   {frc::DoubleSolenoid::kReverse, frc::DoubleSolenoid::kForward}, 
    //   {frc::DoubleSolenoid::kForward, frc::DoubleSolenoid::kReverse}, 
    // };

    // constexpr std::unordered_map<frc::DoubleSolenoid::Value, frc::DoubleSolenoid::Value> reverseStates = {
    //   {frc::DoubleSolenoid::kReverse, frc::DoubleSolenoid::kForward}, 
    //   {frc::DoubleSolenoid::kForward, frc::DoubleSolenoid::kReverse}, 
    // };
};