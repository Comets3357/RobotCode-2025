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
#include "frc2/command/CommandPtr.h"
#include "frc/DriverStation.h"

void AutonCommands(DriveSubsystem* m_drive, ClimbSubsystem* m_climb, ElevatorSubsystem* m_elevator,
                    ElbowSubsystem* m_elbow, IntakeSubsystem* m_intake, LEDSubsystem* m_LED);

frc2::CommandPtr GoToAndScore(
    frc::Pose2d targetPose, DriveSubsystem* m_drive, ElbowSubsystem* m_elbow, ElevatorSubsystem* m_elevator,
    units::second_t bufferTime = 3.0_s, units::meter_t MOE = 0.03_m,  units::degree_t MOErotation = 1.5_deg); 


