// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include "Commands/DefaultElbowCommand.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

  elbowSubsystem.SetDefaultCommand(DefaultElbowCommand(&elbowSubsystem, 
  [this] { return driverController.GetRightX(); 
  }).ToPtr());



}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
