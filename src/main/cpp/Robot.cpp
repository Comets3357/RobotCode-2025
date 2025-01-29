#include "Robot.h"

#include <frc2/command/CommandScheduler.h>


Robot::Robot() {
  frc::SmartDashboard::PutData("Field", &m_field);
}


void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
    // Do this in either robot or subsystem init
    frc::SmartDashboard::PutData("Field", &m_field);
    // Do this in either robot periodic or subsystem periodic
    m_field.SetRobotPose(m_container.m_drive.GetPose());

    
    photon::PhotonTrackedTarget target = cameraResults.GetBestTarget();
    frc::Transform3d pose = target.GetBestCameraToTarget();

    double distanceX = (double) pose.X(); 
    double distanceY = (double) pose.Y();
    double distanceZ = (double) pose.Z();

    double distance = sqrt(pow(distanceX, 2) + pow(distanceY, 2) + pow(distanceZ, 2));


    frc::SmartDashboard::PutBoolean("Has a target", cameraResults.HasTargets());
    // frc::SmartDashboard::PutNumber("Which AprilTag", target.GetFiducialId());
    // frc::SmartDashboard::PutNumber("Distance X", distanceX);
    // frc::SmartDashboard::PutNumber("Distance Y", distanceY);
    // frc::SmartDashboard::PutNumber("Distance Z", distanceZ);
    frc::SmartDashboard::PutNumber("Yaw", target.GetYaw());
    frc::SmartDashboard::PutNumber("Pitch", target.GetPitch());
    frc::SmartDashboard::PutNumber("Skew/Roll?", target.GetSkew());
    frc::SmartDashboard::PutNumber("Rotation", pose.Rotation().Angle().value());

prevEstimatedRobotPose = getEstimatedGlobalPose(prevEstimatedRobotPose);
frc::SmartDashboard::PutNumber("Distance X", prevEstimatedRobotPose.X().value());
frc::SmartDashboard::PutNumber("Distance Y", prevEstimatedRobotPose.Y().value());
frc::SmartDashboard::PutNumber("Distance Z", prevEstimatedRobotPose.Z().value());

m_field.SetRobotPose(getEstimatedGlobalPose(prevEstimatedRobotPose).ToPose2d());

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
