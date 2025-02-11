#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include "grpl/CanBridge.h"



Robot::Robot() {
      grpl::start_can_bridge();
}

void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
    // Do this in either robot or subsystem init
    frc::SmartDashboard::PutData("Field", &m_field);
    // Do this in either robot periodic or subsystem periodic
    //m_field.SetRobotPose(m_container.m_drive.GetPose());
    //m_container.m_drive.UpdateOdometry();

    // frc::SmartDashboard::PutNumber("robot pose X", m_container.m_drive.m_poseEstimator.GetEstimatedPosition().Translation().X().value());
    // frc::SmartDashboard::PutNumber("robot pose Y", m_container.m_drive.m_poseEstimator.GetEstimatedPosition().Translation().Y().value());
    // frc::SmartDashboard::PutNumber("odometry X", m_container.m_drive.m_odometry.GetPose().X().value());
    // frc::SmartDashboard::PutNumber("odometry Y", m_container.m_drive.m_odometry.GetPose().Y().value());
    if (m_container.m_visionSubsystem.getEstimatedGlobalPose(m_container.m_visionSubsystem.prevEstimatedRobotPose).size() > 0) {
        m_container.m_visionSubsystem.prevEstimatedRobotPose = m_container.m_visionSubsystem.getEstimatedGlobalPose(m_container.m_visionSubsystem.prevEstimatedRobotPose).at(0);
        frc::SmartDashboard::PutNumber("Distance X", m_container.m_visionSubsystem.prevEstimatedRobotPose.X().value());
        frc::SmartDashboard::PutNumber("Distance Y", m_container.m_visionSubsystem.prevEstimatedRobotPose.Y().value());
        frc::SmartDashboard::PutNumber("Distance Z", m_container.m_visionSubsystem.prevEstimatedRobotPose.Z().value());
    }

    if (m_container.m_visionSubsystem.EstimatedPose().has_value()) {
    frc::SmartDashboard::PutNumber("hail mary X", (double) m_container.m_visionSubsystem.EstimatedPose().value().estimatedPose.X());
    }
      std::optional<grpl::LaserCanMeasurement> measurement = testLaserCan.get_measurement();
  if (measurement.has_value() && measurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
    frc::SmartDashboard::PutNumber("LaserCAN distance", measurement.value().distance_mm);
  } else {
    std::cout << "Oh no! The target is out of range, or we can't get a reliable measurement!" << std::endl;
    // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
  }


}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit()
{
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{
    if (m_autonomousCommand)
    {
        m_autonomousCommand->Cancel();
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
