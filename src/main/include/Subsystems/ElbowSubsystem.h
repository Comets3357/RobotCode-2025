#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>

class ElbowSubsystem : public frc2::SubsystemBase {
public:

    ElbowSubsystem();

    //constants
    
    
    //epic PID system for the elbow, dont change these values unless you know what you are doing :)
    const double elbowP = 1;
    const double elbowI = 0;
    const double elbowD = 0;
    frc::PivotPIDController{elbowP, elbowI, elbowD};

    //targetAngle, pretty self-explainatory if you ask me.
    int targetAngle = 0;
    
    //setters
    void setElbowAngle(int degree);
    void setElbowSpeed(int speed);
    
    //getters
    void getElbowAngle();
    void getElbowSpeed();

private:


}