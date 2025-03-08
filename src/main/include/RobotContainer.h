// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ElbowSubsystem.h"
#include "subsystems/ClimbSubsystem.h"
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/ElevatorSubsystem.h"
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <utility>
#include "Constants.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <memory>


#include "subsystems/LEDSubsystem.h"


class RobotContainer
{
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();
    DriveSubsystem m_drive;
    ElevatorSubsystem m_elevator;
    IntakeSubsystem intake;
    ElbowSubsystem m_elbowSubsystem;
    ClimbSubsystem m_climb;
    LEDSubsystem m_LED{&m_climb, &m_drive}; 
    
    //frc::SendableChooser<frc2::Command*> autoChooser = pathplanner::AutoBuilder::buildAutoChooser(); 
    //double trimVar = 0;

private:
    // The driver's controller
    frc2::CommandXboxController m_driverController{OIConstants::kDriverControllerPort};
    frc2::CommandXboxController m_secondaryController{OIConstants::kSecondaryControllerPort};
    bool halfSpeed = false; 
    frc::SendableChooser<frc2::Command *> autoChooser;

    // The robot's subsystems and commands are defined here...
    // The robot's subsystems
    // The chooser for the autonomous routines
    // frc::SendableChooser<frc2::Command*> m_chooser;
    void ConfigureButtonBindings();

    int offset = 90;

    //double trimOffset = 0; 
};