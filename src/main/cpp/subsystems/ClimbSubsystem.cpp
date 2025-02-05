#include "Subsystems/ClimbSubsystem.h"

void ClimbSubsystem::ClimbRetract()
{
    ClimbMotor.SetPercent(0.2);
}

void ClimbSubsystem::ClimbStop(){
    ClimbMotor.SetPercent(0);
}

void ClimbSubsystem::ClimbSetPercent(double percent){
    ClimbMotor.SetPercent(percent);
}

void ClimbSubsystem::ClimbSetPosition(double position){
    ClimbMotor.setReference(position, Motor::controlType::position);
}

bool ClimbSubsystem::ClimbMax()
{
    ClimbMotor.enableForwardSoftLimit(true);
    ClimbMotor.setForwardSoftLimit(0.01);
}

ClimbSubsystem::ClimbSubsystem(){
    ClimbMotor.SetSmartCurrentLimit(20);
    ClimbMotor.setFeedbackSensor(Motor::encoderType::absolute);
    ClimbMotor.setAbsolutePositionConversionFactor(360);
    ClimbMotor.setAbsoluteVelocityConversionFactor(6);
    ClimbMotor.setReverseSoftLimit(90);
    ClimbMotor.setForwardSoftLimit(180);
    ClimbMotor.enableForwardSoftLimit(true);
    ClimbMotor.enableReverseSoftLimit(true);
    ClimbMotor.configure();
}
