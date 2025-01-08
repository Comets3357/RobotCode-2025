#include "subsystems/ElbowSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

ElbowSubsystem::ElbowSubsystem() {

    elbowPivotMotor.setAbsolutePositionConversionFactor(360 /*degrees*/);
    elbowPivotMotor.setForwardSoftLimit(60);
    elbowPivotMotor.setReverseSoftLimit(30);

    elbowPivotMotor.setPID(elbowP, elbowI, elbowD);

}

//setters

//sets the elbow angle (360 degrees)
void ElbowSubsystem::setTargetElbowAngle(int angle) {
    targetAngle = angle;
}

//sets the speed in percent
void ElbowSubsystem::setElbowSpeed(int speed) {
    elbowPivotMotor.SetPercent(speed);
}

//sets the gripper state between active and not active
void ElbowSubsystem::setGripperState(bool state) {
    isGripperToggled = state;
}

//switches the state between on and off for the gripper
void ElbowSubsystem::toggleGripper() {
    isGripperToggled = !isGripperToggled;
}

//getters

//gets the elbow angle in absolute position
double ElbowSubsystem::getElbowAngle() {
    return elbowPivotMotor.GetAbsolutePosition();
}

//gets the target angle
double ElbowSubsystem::getTargetAngle() {
    return targetAngle;
}

//gets the velocity of the elbow pivot motor
double ElbowSubsystem::getElbowSpeed() {
    return elbowPivotMotor.GetAbsoluteVelocity();
}

//gets the state of the arm gripper
double ElbowSubsystem::getGripperState() {
    return isGripperToggled;
}