#include "subsystems/ElevatorSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

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
        MainElevatorMotor.setAbsolutePositionConversionFactor(56.7);
        MainElevatorMotor.setAbsoluteVelocityConversionFactor(0.945);//in/motor rotation .021 in per sec
        MainElevatorMotor.setAbsoluteEncoderInverted(true);
        FollowElevatorMotor.setAbsolutePositionConversionFactor(56.7);
        FollowElevatorMotor.setAbsoluteVelocityConversionFactor(0.945);
        MainElevatorMotor.SetSmartCurrentLimit(50);
        FollowElevatorMotor.SetSmartCurrentLimit(50);
        MainElevatorMotor.setPID(elevatorP, elevatorI, elevatorD);
        FollowElevatorMotor.setPID(elevatorP, elevatorI, elevatorD);
        FollowElevatorMotor.setInverted(false);
        MainElevatorMotor.setInverted(true);
        MainElevatorMotor.enableForwardSoftLimit(true);
        MainElevatorMotor.enableReverseSoftLimit(true);
        FollowElevatorMotor.enableForwardSoftLimit(false);
        FollowElevatorMotor.enableReverseSoftLimit(false);
        MainElevatorMotor.setForwardSoftLimit(50.5);
        MainElevatorMotor.setReverseSoftLimit(3);
        MainElevatorMotor.setMinOutput(-0.5);
        MainElevatorMotor.setMaxOutput(0.5);
        FollowElevatorMotor.SetFollow(MainElevatorMotor);
        MainElevatorMotor.setFeedbackSensor(Motor::encoderType::absolute);
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

double ElevatorSubsystem::getRPosition()
{
    return MainElevatorMotor.GetRelativePosition();
}

double ElevatorSubsystem::getAPosition()
{
    return MainElevatorMotor.GetAbsolutePosition();
}


// void ElevatorSubsystem::setPidSlot(int n)
// {
//     if (n == 0)
//     {}
//         MainElevatorMotor.setPID(elevatorP, elevatorI, elevatorD, 0, 0);

// }

void ElevatorSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("elevatorPos", MainElevatorMotor.GetAbsolutePosition());
}


