#pragma once

#include "Subsystems/ClimbSubsystem.h"
#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/ElbowSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/LEDSubsystem.h"
#include "Subsystems/MAXSwerveModule.h"

#include "commands/IntakeCommands.h"
#include <frc2/command/button/CommandXboxController.h>

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <frc/DriverStation.h>

void DriverCommands(DriveSubsystem* m_drive, ClimbSubsystem* m_climb, ElevatorSubsystem* m_elevator,
                    ElbowSubsystem* m_elbowSubsystem, IntakeSubsystem* m_intake, LEDSubsystem* m_LED, 
                    frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController);


frc2::CommandPtr rotateTo(DriveSubsystem *drive, units::degree_t targetrot, frc2::CommandXboxController *m_driverController);

double shortestRotation(double current, double target);
