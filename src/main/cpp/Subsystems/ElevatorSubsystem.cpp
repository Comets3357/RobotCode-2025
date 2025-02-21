#include "Subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() 
{
    MainElevatorMotor.setRelativePositionConversionFactor(1.260)
    MainElevatorMotor.GetRelativeVelocity(0.021) //in/motor rotation .021 in per sec
    FollowElevatorMotor.setRelativePositionConversionFactor(1.260)
    FollowElevatorMotor.GetRelativeVelocity(0.021)
    MainElevatorMotor.setPID(elevatorP, elevatorI, elevatorD);
    FollowElevatorMotor.setPID(elevatorP, elevatorI, elevatorD);
    FollowElevatorMotor.setInverted(true);
    MainElevatorMotor.setInverted(false);
    MainElevatorMotor.setForwardSoftLimit(25);
    FollowElevatorMotor.setForwardSoftLimit(25)
    MainElevatorMotor.configure();
    FollowElevatorMotor.configure();

}

ElevatorSubsystem::getPosition(double TargetPosition)
{
    double target = TargetPosition;
}

ElevatorSubsystem::setSpeed(int speed)
{
    MainElevatorMotor.SetPercent(speed);
    FollowElevatorMotor.SetPercent(speed);
}

void ElevatorSubsystem::ElevatorExtend()
{
    MainElevatorMotor.SetPercent(0.02);
    FollowElevatorMotor.SetPercent(0.02);
}

void ElevatorSubsystem::ElevatorRetract()
{
    MainElevatorMotor.SetPercent(-0.02);
    FollowElevatorMotor.SetPercent(-0.02);
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