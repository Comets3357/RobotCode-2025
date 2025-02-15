#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
    // Do this in either robot or subsystem init
    frc::SmartDashboard::PutData("Field", &m_field);
    // Do this in either robot periodic or subsystem periodic
    m_field.SetRobotPose(m_container.m_drive.GetPose());

    frc::SmartDashboard::PutNumber("Test Speed", m_container.testspeed * 4.8); 
    frc::SmartDashboard::PutNumber("Heading", (double)m_container.m_drive.GetGyroHeading().Degrees());
    //frc::SmartDashboard::PutNumber("Acutal Speed", (double)m_container.m_drive.GetRobotRelativeSpeeds().vx); 
    frc::SmartDashboard::PutNumber("Single Wheel Chassis Speed", (double)m_container.m_drive.GetChassisSpeed());
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
