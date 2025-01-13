#include "Subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() {}

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

// bool ElevatorSubsystem::ElevatorMax()
// {
//     MainElevatorMotor.GetAbsolutePosition();
// }
