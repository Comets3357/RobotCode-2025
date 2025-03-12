#pragma once

#include "Subsystems/ElbowSubsystem.h"
#include "frc2/command/CommandPtr.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "RobotContainer.h"

frc2::CommandPtr WristStuff(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, double idle);