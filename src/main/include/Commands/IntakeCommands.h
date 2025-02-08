#pragma once

#include "Subsystems/IntakeSubsystem.h"
#include "frc2/command/CommandPtr.h"
#include <units/angle.h>

frc2::CommandPtr IntakeAlgae(IntakeSubsystem* intake);
frc2::CommandPtr DeployAlgae(IntakeSubsystem* intake);
frc2::CommandPtr StopIntake(IntakeSubsystem* intake);
frc2::CommandPtr StopDeploy(IntakeSubsystem* intake);