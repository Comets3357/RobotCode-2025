#pragma once

#include "Subsystems/ElevatorSubsystem.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

class ElevatorCommand : public frc2::CommandHelper<frc2::Command, ElevatorCommand>
{
    public:
        explicit ElevatorCommand(ElevatorSubsystem* ElevatorSubsystem);
        void Execute();
    private:
        ElevatorSubsystem* elevator;
};