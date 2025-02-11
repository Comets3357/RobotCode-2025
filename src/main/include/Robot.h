// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"
#include "grpl/LaserCan.h"


class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

  double getHorizontalDistanceMeasurement();
  double getVerticalDistanceMeasurement();
  std::optional<grpl::LaserCanMeasurement> vertMeasurement;
  std::optional<grpl::LaserCanMeasurement> horizMeasurement;

 private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  RobotContainer m_container;
  grpl::LaserCan LaserCanVertical{19};
  grpl::LaserCan LaserCanHorizontal{20};



};
