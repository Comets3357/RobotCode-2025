#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystems/ElbowSubsystem.h"

class DefaultElbowCommand
    : public frc2::CommandHelper<frc2::Command, DefaultElbowCommand> {
 public:
    std::function<double()> rightStickSupplier;
    std::function<double()> rightStick;

    std::function<double()> rightTrigger;
    std::function<double()> rightTriggerSupplier;

    /**
    * Creates a new ElbowCommand (yippee!).
    *
    * @param pivotSubsystem The subsystem used by this command.
    * @param supplier POV
    */
    explicit DefaultElbowCommand(ElbowSubsystem* elbowSubsystem, std::function<double()> rightStick, std::function<double()> rightTrigger);

   static frc2::CommandPtr setGripperPos(ElbowSubsystem *m_elbowSubsystem, double position);

   static frc2::CommandPtr setIdle(ElbowSubsystem *m_elbowSubsystem);
   static frc2::CommandPtr setIntake(ElbowSubsystem *m_elbowSubsystem);
   static frc2::CommandPtr setOuttake(ElbowSubsystem *m_elbowSubsystem);

    void Execute() override;

 private:

  ElbowSubsystem* elbowSubsystem;
};