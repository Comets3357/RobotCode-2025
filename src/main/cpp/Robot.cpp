#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include "grpl/CanBridge.h"

Robot::Robot() {
    grpl::start_can_bridge();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();


std::optional<grpl::LaserCanMeasurement> vertMeasurement = LaserCanVertical.get_measurement();
std::optional<grpl::LaserCanMeasurement> horizMeasurement = LaserCanHorizontal.get_measurement();


  if (vertMeasurement.has_value() && vertMeasurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
    frc::SmartDashboard::PutNumber("Vertical LaserCAN Measurement", vertMeasurement.value().distance_mm);
  } else {
    std::cout << "no vertical measurement" << std::endl;
  }
    if (horizMeasurement.has_value() && horizMeasurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
    frc::SmartDashboard::PutNumber("Horizontal LaserCAN Measurement", horizMeasurement.value().distance_mm);
  } else {
    std::cout << "no horizontal measurement" << std::endl;
  }
}

double Robot::getHorizontalDistanceMeasurement() {
  return horizMeasurement.value().distance_mm;
}

double Robot::getVerticalDistanceMeasurement() {
  return vertMeasurement.value().distance_mm;
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
