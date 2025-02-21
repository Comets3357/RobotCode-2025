#include "Subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() 
{
    if (CompBotSettings==false)
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
    else
    {
        MainElevatorMotor.setRelativePositionConversionFactor(56.7);
        MainElevatorMotor.setRelativeVelocityConversionFactor(0.0945);//in/motor rotation .021 in per sec
        FollowElevatorMotor.setRelativePositionConversionFactor(1);
        FollowElevatorMotor.setRelativeVelocityConversionFactor(1);
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
    
}

void ElevatorSubsystem::setSpeed(double speed)
{
    MainElevatorMotor.SetPercent(speed);
}

void ElevatorSubsystem::setPosition(double position)
{
    MainElevatorMotor.setReference(position, Motor::controlType::position);
}

double ElevatorSubsystem::getPosition()
{
    return MainElevatorMotor.GetRelativePosition();
}
void ElevatorSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("elevatorPos", MainElevatorMotor.GetRelativePosition());
}
