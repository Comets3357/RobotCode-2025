#include <Subsystems/ElevatorSubsystem.h>


void ElevatorSubsystem::ElevatorExtend()
{
    MainElevatorMotor.Set(0.2);
}

void ElevatorSubsystem::ElevatorStop()
{
    MainElevatorMotor.Set(0);
}

void ElevatorSubsystem::ElevatorLower()
{
    MainElevatorMotor.Set(-0.2);
}

