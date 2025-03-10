#pragma once

#include "Subsystems/IntakeSubsystem.h"
#include "frc2/command/CommandPtr.h"
#include <units/angle.h>

frc2::CommandPtr IntakeAlgae(IntakeSubsystem* m_intake);
frc2::CommandPtr DeployAlgae(IntakeSubsystem* m_intake);
frc2::CommandPtr StopIntake(IntakeSubsystem* m_intake);
frc2::CommandPtr StopDeploy(IntakeSubsystem* m_intake);