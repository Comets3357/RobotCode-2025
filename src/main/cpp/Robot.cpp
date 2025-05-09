#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotPeriodic()
{

    frc2::CommandScheduler::GetInstance().Run();
    // Do this in either robot or subsystem init
    // frc::SmartDashboard::PutData("Field", &m_field);
    // Do this in either robot periodic or subsystem periodic
    // m_field.SetRobotPose(m_container.m_drive.GetPose());
    // m_container.m_drive.PoseEstimation();

    // double chassisSpeedSquared= pow((double)m_container.m_drive.GetRobotRelativeSpeeds().vx, 2) + pow((double)m_container.m_drive.GetRobotRelativeSpeeds().vy, 2); 
    // double chassisSpeeds = pow(chassisSpeedSquared, 0.5); 

    // frc::SmartDashboard::PutNumber("Heading", (double)m_container.m_drive.GetGyroHeading().Degrees());
    // frc::SmartDashboard::PutNumber("Acutal Speed", chassisSpeeds); 
    // frc::SmartDashboard::PutNumber("Single Wheel Chassis Speed", (double)m_container.m_drive.GetChassisSpeed());

    // frc::SmartDashboard::PutNumber("X", (double)m_container.m_drive.GetPose().X()); 
    // frc::SmartDashboard::PutNumber("Y", (double)m_container.m_drive.GetPose().Y()); 

    // frc::SmartDashboard::PutNumber("Degrees Rotation", (double)m_container.m_drive.GetPose().Rotation().Degrees()); 

    //  frc::SmartDashboard::PutNumber("robot pose X", m_container.m_drive.m_poseEstimator.GetEstimatedPosition().Translation().X().value());
    // frc::SmartDashboard::PutNumber("robot pose Y", m_container.m_drive.m_poseEstimator.GetEstimatedPosition().Translation().Y().value());
    // frc::SmartDashboard::PutNumber("robot pose heading", m_container.m_drive.m_poseEstimator.GetEstimatedPosition().Translation().Angle().Degrees().value());
    // frc::SmartDashboard::PutNumber("estimated pose vector value", m_container.m_drive.estimatedPoseVector.size());


    if(!m_digitalInputZero.Get()){
        m_container.m_drive.ZeroHeading();
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
