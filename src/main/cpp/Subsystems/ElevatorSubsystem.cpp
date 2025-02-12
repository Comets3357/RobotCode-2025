#include "Subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() 
{
    MainElevatorMotor.setRelativePositionConversionFactor(1.260);
    MainElevatorMotor.setRelativeVelocityConversionFactor(0.021);//in/motor rotation .021 in per sec
    FollowElevatorMotor.setRelativePositionConversionFactor(1.260);
    FollowElevatorMotor.setRelativeVelocityConversionFactor(0.021);
    MainElevatorMotor.SetSmartCurrentLimit(30);
    FollowElevatorMotor.SetSmartCurrentLimit(30);
    MainElevatorMotor.setPID(elevatorP, elevatorI, elevatorD);
    FollowElevatorMotor.setPID(elevatorP, elevatorI, elevatorD);
    FollowElevatorMotor.setInverted(true);
    MainElevatorMotor.setInverted(false);
    MainElevatorMotor.configure();
    FollowElevatorMotor.configure();
}

void ElevatorSubsystem::setSpeed(double speed)
{
    MainElevatorMotor.SetPercent(speed);
    FollowElevatorMotor.SetPercent(speed);
}

void ElevatorSubsystem::setPosition(double position)
{
    MainElevatorMotor.setReference(position, Motor::controlType::position);
}

// void ElevatorSubsystem::ElevatorExtend()// {
//     MainElevatorMotor.SetPercent(0.02);
//     FollowElevatorMotor.SetPercent(0.02);
// }
// void ElevatorSubsystem::getPosition(double TargetPosition)
// {
//     double target = TargetPosition;
// }
// void ElevatorSubsystem::ElevatorRetract()
// {
//     MainElevatorMotor.SetPercent(-0.02);
//     FollowElevatorMotor.SetPercent(-0.02);
// }
// void ElevatorSubsystem::ElevatorStop()
// {
//     MainElevatorMotor.StopMotor();
//     FollowElevatorMotor.StopMotor();
// }
// void ElevatorSubsystem::ElevatorMax()
// {
//     MainElevatorMotor.enableForwardSoftLimit(true);
//     MainElevatorMotor.setForwardSoftLimit(0.01);
//     FollowElevatorMotor.enableForwardSoftLimit(true);
//     FollowElevatorMotor.setForwardSoftLimit(0.01);
// }