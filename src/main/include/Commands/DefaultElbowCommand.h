#pragma once

#include <frc2/command/CommandPtr.h>
#include "Subsystems/ElbowSubsystem.h"

class DefaultElbowCommand
    : public frc2::CommandHelper<frc2::Command, DefaultElbowCommand> {
 public:
    std::function<double()> POVSupplier;
    std::function<double()> POV;

    /**
    * Creates a new ElbowCommand (yippee!).
    *
    * @param pivotSubsystem The subsystem used by this command.
    * @param supplier POV
    */
    explicit DefaultElbowCommand(ElbowSubsystem* elbowSubsystem, std::function<double()> POV);

    void Execute() override;

 private:
  ElbowSubsystem* elbowSubsystem;
};