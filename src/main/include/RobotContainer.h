// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/ElevatorSubsystem.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <units/angle.h>

using namespace pathplanner;


class RobotContainer
{
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();
    VisionSubsystem m_visionSubsystem;
    DriveSubsystem m_drive;
    // ElevatorSubsystem m_elevator; 

    frc::Pose2d targetPose = frc::Pose2d(10_m, 5_m, frc::Rotation2d(180_deg));

// Create the constraints to use while pathfinding
PathConstraints constraints = PathConstraints(
    3.0_mps, 4.0_mps_sq,
    units::degrees_per_second_t{540}, units::degrees_per_second_squared_t{720});

// Since AutoBuilder is configured, we can use it to build pathfinding commands
frc2::CommandPtr pathfindingCommand = AutoBuilder::pathfindToPose(
    targetPose,
    constraints,
    0.0_mps // Goal end velocity in meters/sec
);

// frc2::CommandPtr pathdostuff = AutoBuilder::pathfindThenFollowPath(
//     pathfindingCommand, 
//     constraints
// );

    
    

private:
    // The driver's controller
    frc2::CommandXboxController m_driverController{OIConstants::kDriverControllerPort};
   
    // The robot's subsystems and commands are defined here...
    // The robot's subsystems
    // The chooser for the autonomous routines
    // frc::SendableChooser<frc2::Command*> m_chooser;
    void ConfigureButtonBindings();

    // IntakeSubsystem intake;
};
