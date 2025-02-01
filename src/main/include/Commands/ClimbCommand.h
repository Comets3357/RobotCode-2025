#pragma once

#include "Subsystems/ClimbSubsystem.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

frc2::CommandPtr ClimbRetract(ClimbSubsystem* climb);
frc2::CommandPtr ClimbStop(ClimbSubsystem* climb);
frc2::CommandPtr ClimbSet(ClimbSubsystem* climb, double percent);
