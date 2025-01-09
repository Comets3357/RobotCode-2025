// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/XboxController.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"


class RobotContainer {
 public:
  RobotContainer();

  //frc2::CommandPtr GetAutonomousCommand();

 private:

 // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;

  // The chooser for the autonomous routines
  //frc::SendableChooser<frc2::Command*> m_chooser;
  void ConfigureButtonBindings(); 
  void ConfigureBindings();
};
