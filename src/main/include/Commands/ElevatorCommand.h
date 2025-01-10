#pragma once

#include "Subsystems/ElevatorSubsystem.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

frc2::CommandPtr ExtendElevator(ElevatorSubsystem* elevator);
frc2::CommandPtr RetractElevator(ElevatorSubsystem* elevator);
frc2::CommandPtr StopElevator(ElevatorSubsystem* elevator);