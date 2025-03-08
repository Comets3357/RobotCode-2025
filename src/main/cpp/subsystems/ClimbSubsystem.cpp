#include "Subsystems/ClimbSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

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
   // climbMotor.setReference(position, Motor::controlType::position);
    //climbMotor.
    
    if (climbMotor.GetAbsolutePosition() < position) {
        climbMotor.SetPercent(-0.2);
    } else if (climbMotor.GetAbsolutePosition() >= position) {
        climbMotor.SetPercent(0); 
    } else {
        climbMotor.SetPercent(0);
    }
}

void ClimbSubsystem::ClimbMax()
{
    climbMotor.enableForwardSoftLimit(true);
    climbMotor.setForwardSoftLimit(0.01);
}

ClimbSubsystem::ClimbSubsystem(){
    climbMotor.SetSmartCurrentLimit(20);
    climbMotor.setFeedbackSensor(Motor::encoderType::absolute);
    climbMotor.setAbsolutePositionConversionFactor(360);
    climbMotor.setAbsoluteVelocityConversionFactor(6);
    climbMotor.setReverseSoftLimit(2);
    climbMotor.setForwardSoftLimit(120);
    climbMotor.enableForwardSoftLimit(false);
    climbMotor.enableReverseSoftLimit(false);
    climbMotor.setPID(0.02,0,0);
    //climbMotor.setAbsoluteEncoderInverted(true); 
    climbMotor.configure();
    
}

void ClimbSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("climb position", climbMotor.GetAbsolutePosition());
}
