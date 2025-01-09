// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Commands/IntakeCommands.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  controller.A().WhileTrue(IntakeAlgae(&intake));
  controller.A().OnFalse(StopIntake(&intake));

  controller.Y().WhileTrue(DeployAlgae(&intake));
  controller.Y().OnFalse(StopIntake(&intake));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
