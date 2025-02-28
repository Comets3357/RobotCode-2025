#include "Subsystems/ClimbSubsystem.h"

void ClimbSubsystem::ClimbRetract()
{
    climbMotor.SetPercent(0.2);
}

void ClimbSubsystem::ClimbStop(){
    climbMotor.SetPercent(0);
}

void ClimbSubsystem::ClimbSetPercent(double percent){
    climbMotor.SetPercent(percent);
}

void ClimbSubsystem::ClimbSetPosition(double position){
    climbMotor.setReference(position, Motor::controlType::position);
}

bool ClimbSubsystem::ClimbMax()
{
    climbMotor.enableForwardSoftLimit(true);
    climbMotor.setForwardSoftLimit(0.01);
}

ClimbSubsystem::ClimbSubsystem(){
    climbMotor.SetSmartCurrentLimit(20);
    climbMotor.setFeedbackSensor(Motor::encoderType::absolute);
    climbMotor.setAbsolutePositionConversionFactor(360);
    climbMotor.setAbsoluteVelocityConversionFactor(6);
    climbMotor.setReverseSoftLimit(90);
    climbMotor.setForwardSoftLimit(180);
    climbMotor.enableForwardSoftLimit(true);
    climbMotor.enableReverseSoftLimit(true);
    climbMotor.configure();

    climbMotor.setPID(0.02,0,0);
}
