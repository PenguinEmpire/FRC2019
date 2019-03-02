#pragma once

// #include "Robot.h"
#include <unorderedmap>
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
      : frc::DoubleSolenoid(pcm, pch_1, pch_2), in(state)
    {
      Set(state);

      timer->Reset();
      timer->Start();
      // Toggle(); // testing

      busy = false;
    }

    void Toggle() {
      if (!busy) {
        Set(reverseStates[Get()]);
      }
    }

    void Toggle(bool btn) {
      if (btn) {
        Toggle();
      }
    }

    void ToggleOverride() {
      Set(reverseStates[Get()]);
    }

    void ToggleOverride(bool btn) {
      if (btn) {
        ToggleOverride();
      }
    }

    void ToggleTimed(double seconds) {
      printf("called ToggleTimed\n");
      goalTime = timer->Get() + seconds;
      printf("curTime: %f\n", curTime);
      printf("goalTime: %f\n", goalTime);
      printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
      ToggleOverride();
      busy = true;
      // timer->Start();
    }

    void ToggleTimed(double seconds, bool btn) {
      if (btn && !busy) {
        ToggleTimed(seconds);
      }
    }

    void PenguinUpdate() {
      curTime = timer->Get();
      
      printf("calling penguin update; curTime : %f, goalTime : %f, busy : %i \n", curTime, goalTime, busy);

      if (busy) {
        if (timer->Get() >= goalTime) {
          ToggleOverride();
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