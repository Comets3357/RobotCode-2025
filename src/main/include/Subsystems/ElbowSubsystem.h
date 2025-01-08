#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include "wrapperclasses/SparkMaxMotor.h"

class ElbowSubsystem : public frc2::SubsystemBase {
public:

    ElbowSubsystem();

    //constants
    int elbowPivotID = 11;
    int armGripperID = 12;
    
    //epic PID system for the elbow, dont change these values unless you know what you are doing :)
    const double elbowP = 1;
    const double elbowI = 0;
    const double elbowD = 0;

    bool isGripperToggled = false;
    int targetAngle = 0;
    
    //setters

    void setTargetElbowAngle(int angle);
    void setElbowSpeed(int speed);
    void toggleGripper();
    void setGripperState(bool state);
    
    //getters

    double getGripperState();
    double getTargetAngle();
    double getElbowAngle();
    double getElbowSpeed();

    void CalculatePID();

private:
    SparkMaxMotor elbowPivotMotor{elbowPivotID}; 
    SparkMaxMotor gripperMotor{armGripperID};

};