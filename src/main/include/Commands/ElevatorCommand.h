#pragma once

#include "Subsystems/ElevatorSubsystem.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>

// frc2::CommandPtr ExtendElevator(ElevatorSubsystem* elevator);
// frc2::CommandPtr RetractElevator(ElevatorSubsystem* elevator);
// frc2::CommandPtr StopElevator(ElevatorSubsystem* elevator);

class elevatorCommand : public frc2::CommandHelper<frc2::Command, elevatorCommand>
{
    public:
        // std::function<double()> LeftStickSupplier;
        // std::function<double()> LeftStick;

        frc2::CommandXboxController controller{0};

        explicit elevatorCommand(ElevatorSubsystem* elevatorSubsystem);
        void Execute();

    private:
        ElevatorSubsystem* elevatorSubsystem;       
};