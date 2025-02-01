#include "Subsystems/ClimbSubsystem.h"

void ClimbSubsystem::ClimbRetract()
{
    ClimbMotor.SetPercent(0.2);
}

void ClimbSubsystem::ClimbStop(){
    ClimbMotor.SetPercent(0);
}

void ClimbSubsystem::ClimbSet(double percent){
    ClimbMotor.SetPercent(percent);
}

bool ClimbSubsystem::ClimbMax()
{
    ClimbMotor.enableForwardSoftLimit(true);
    ClimbMotor.setForwardSoftLimit(0.01);
}

ClimbSubsystem::ClimbSubsystem(){
    ClimbMotor.configure();
}
