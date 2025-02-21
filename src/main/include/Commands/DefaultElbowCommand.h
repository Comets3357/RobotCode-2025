#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/ElbowSubsystem.h"

class DefaultElbowCommand
    : public frc2::CommandHelper<frc2::Command, DefaultElbowCommand> {
 public:

   static frc2::CommandPtr setWristPos(ElbowSubsystem *m_elbowSubsystem, double position);
   static frc2::CommandPtr setElbowPos(ElbowSubsystem *m_elbowSubsystem, double position);

   static frc2::CommandPtr setWristSpeed(ElbowSubsystem *m_elbowSubsystem, double speed);
   static frc2::CommandPtr setElbowSpeed(ElbowSubsystem *m_elbowSubsystem, double speed);
   static frc2::CommandPtr setRollerSpeed(ElbowSubsystem *m_elbowSubsystem, double speed);


    void Execute() override;

 private:

  ElbowSubsystem* elbowSubsystem;
};