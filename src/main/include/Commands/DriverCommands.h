#pragma once

#include "Subsystems/ClimbSubsystem.h"
#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/ElbowSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/LEDSubsystem.h"
#include "Subsystems/MAXSwerveModule.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "commands/IntakeCommands.h"

void DriverCommands(DriverSubsystem m_drive, ClimbSubsystem m_climb, ElevatorSubsystem m_elevator,
                    ElbowSubsystem m_elbowSubsystem, IntakeSubsystem m_intake, LEDSubsystem m_LED);

