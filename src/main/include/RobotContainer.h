// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include <frc2/command/button/CommandXboxController.h>

class RobotContainer
{
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

    DriveSubsystem m_drive;
    double testspeed = 2.7 / 4.8; 

private:
    // The driver's controller
    frc2::CommandXboxController m_driverController{OIConstants::kDriverControllerPort};

    // The robot's subsystems and commands are defined here...

    // The robot's subsystems

    // The chooser for the autonomous routines
    // frc::SendableChooser<frc2::Command*> m_chooser;
    void ConfigureButtonBindings();
    void ConfigureBindings();
};


/*

Test speed:    1         1                1 
PID:           p = 0.04 f = 0.03 0.035    0.038
Actual Speed: 1.54       0.8     0.95     1.02

*/