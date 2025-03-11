#pragma once

#include "Subsystems/ElbowSubsystem.h"
#include "frc2/command/CommandPtr.h"
#include <units/angle.h>
#include "Subsystems/ClimbSubsystem.h"
#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/ElbowSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/LEDSubsystem.h"
#include "Subsystems/MAXSwerveModule.h"
#include <frc2/command/button/CommandXboxController.h>

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>


// frc2::CommandPtr IntakeAlgae(IntakeSubsystem* m_intake);
// frc2::CommandPtr DeployAlgae(IntakeSubsystem* m_intake);
// frc2::CommandPtr StopIntake(IntakeSubsystem* m_intake);
// frc2::CommandPtr StopDeploy(IntakeSubsystem* m_intake);

frc2::CommandPtr WristStuff(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController);