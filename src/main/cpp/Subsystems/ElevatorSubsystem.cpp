#include "Subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() 
{
    MainElevatorMotor.setRelativePositionConversionFactor(1.260/3);
    MainElevatorMotor.setRelativeVelocityConversionFactor(0.021/3);//in/motor rotation .021 in per sec
    FollowElevatorMotor.setRelativePositionConversionFactor(1.260/3);
    FollowElevatorMotor.setRelativeVelocityConversionFactor(0.021/3);
    MainElevatorMotor.SetSmartCurrentLimit(50);
    FollowElevatorMotor.SetSmartCurrentLimit(50);
    MainElevatorMotor.setPID(elevatorP, elevatorI, elevatorD);
    FollowElevatorMotor.setPID(elevatorP, elevatorI, elevatorD);
    FollowElevatorMotor.setInverted(true);
    MainElevatorMotor.setInverted(false);
    MainElevatorMotor.enableForwardSoftLimit(true);
    MainElevatorMotor.enableReverseSoftLimit(true);
    MainElevatorMotor.setForwardSoftLimit(40);
    MainElevatorMotor.setReverseSoftLimit(0);
    MainElevatorMotor.setMinOutput(-0.5);
    MainElevatorMotor.setMaxOutput(0.5);
    FollowElevatorMotor.SetFollow(MainElevatorMotor);
    MainElevatorMotor.configure();
    FollowElevatorMotor.configure();
}

void ElevatorSubsystem::setSpeed(double speed)
{
    MainElevatorMotor.SetPercent(speed);
}

void ElevatorSubsystem::setPosition(double position)
{
    MainElevatorMotor.setReference(position, Motor::controlType::position);
}

void ElevatorSubsystem::getPosition()
{
    MainElevatorMotor.GetRelativePosition();
}
void ElevatorSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("elevatorPos", MainElevatorMotor.GetRelativePosition());
}


// void ElevatorSubsystem::ElevatorExtend()// {
//     MainElevatorMotor.SetPercent(0.02);
//     FollowElevatorMotor.SetPercent(0.02);
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