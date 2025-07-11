#pragma once


#include "Subsystems/DriveSubsystem.h"

#include "Subsystems/MAXSwerveModule.h"


#include <frc2/command/button/CommandXboxController.h>

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/MathUtil.h>
#include <frc/filter/SlewRateLimiter.h>



#include <frc/DriverStation.h>

void DriverCommands(DriveSubsystem* m_drive,  
                    frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController);


frc2::CommandPtr rotateTo(DriveSubsystem *drive, units::degree_t targetrot, frc2::CommandXboxController *m_driverController);

double shortestRotation(double current, double target);

    // Apply a slew rate filter to the commanded speeds

