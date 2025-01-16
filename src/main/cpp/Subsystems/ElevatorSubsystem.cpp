#include "Subsystems/ElevatorSubsystem.h"

void ElevatorSubsystem::ElevatorExtend()
{
    MainElevatorMotor.SetPercent(0.2);
    FollowElevatorMotor.SetPercent(0.2);
}

void ElevatorSubsystem::ElevatorRetract()
{
    MainElevatorMotor.SetPercent(-0.2);
    FollowElevatorMotor.SetPercent(-0.2);
}

void ElevatorSubsystem::ElevatorStop()
{
    MainElevatorMotor.StopMotor();
    FollowElevatorMotor.StopMotor();
}

bool ElevatorSubsystem::ElevatorMax()
{
    MainElevatorMotor.enableForwardSoftLimit(true);
    MainElevatorMotor.setForwardSoftLimit(0.01);
    FollowElevatorMotor.enableForwardSoftLimit(true);
    FollowElevatorMotor.setForwardSoftLimit(0.01);
}

ElevatorSubsystem::ElevatorSubsystem() 
{
    MainElevatorMotor.configure();
    FollowElevatorMotor.configure();
}