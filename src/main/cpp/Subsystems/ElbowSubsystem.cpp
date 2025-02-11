#include "subsystems/ElbowSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "wrapperclasses/SparkMaxMotor.h"
#include "Subsystems/ElbowSubsystem.h"

ElbowSubsystem::ElbowSubsystem() {

    //elbow stuff
    elbowPivotMotor.setRelativeVelocityConversionFactor(4.5);
    elbowPivotMotor.enableForwardSoftLimit(true);
    elbowPivotMotor.enableReverseSoftLimit(true);
    elbowPivotMotor.setForwardSoftLimit(130);
    elbowPivotMotor.setReverseSoftLimit(-120);  
    elbowPivotMotor.SetSmartCurrentLimit(8);  

    gripperPivotMotor.setAbsolutePositionConversionFactor(360 /* Degrees */);
    elbowPivotMotor.SetSmartCurrentLimit(20);

    elbowPivotMotor.setFeedbackSensor(Motor::encoderType::absolute);
    gripperPivotMotor.setFeedbackSensor(Motor::encoderType::absolute);


    elbowPivotMotor.setPID(elbowP, elbowI, elbowD);
    gripperPivotMotor.setPID(gripperP, gripperI, gripperD);

    elbowPivotMotor.configure();
    gripperPivotMotor.configure();

}

//setters

//sets the elbow angle (360 degrees)
void ElbowSubsystem::setTargetElbowAngle(int angle) {
    elbowTargetAngle = angle;
}

//sets the speed in percent
void ElbowSubsystem::setElbowSpeed(double speed) {
    elbowPivotMotor.SetPercent(speed);
}

//sets the current state of an enumerable for the 3 actions of the gripper.
void ElbowSubsystem::setGripperState(ElbowSubsystem::gripperStates state) {
    //conditional to prevent funny stuff with the string system from happening
   if (state == gripperStates::OUTTAKE) {
    gripperState = gripperStates::OUTTAKE;
   } else if (state == gripperStates::INTAKE) {
    gripperState = gripperStates::INTAKE;
   } else {
    gripperState = gripperStates::IDLE;
   }
}

void ElbowSubsystem::setGripperSpeed(double speed) {
    gripperMotor.SetPercent(speed);
}

void ElbowSubsystem::setGripperPivotState(bool state) {
    if (state) {
        gripperPivotState = true;
    } else if (!state) {
        gripperPivotState = false;
    }
}

void ElbowSubsystem::toggleGripperPivotState() {
    gripperPivotState = !gripperPivotState;
}

void ElbowSubsystem::setGripperPivotAngle(double angle) {
    gripperTargetAngle = angle;
}
void ElbowSubsystem::setGripperPivotSpeed(double speed) {
    gripperPivotMotor.SetPercent(speed);
}

//getters

//gets the current state of an enumerable for the 3 actions of the gripper.
ElbowSubsystem::gripperStates ElbowSubsystem::getGripperState() {
    return gripperState;
}

//gets the elbow angle in absolute position
double ElbowSubsystem::getElbowAngle() {
    return elbowPivotMotor.GetAbsolutePosition();
}

//gets the target angle
double ElbowSubsystem::getElbowTargetAngle() {
    return elbowTargetAngle;
}

//gets the velocity of the elbow pivot motor
double ElbowSubsystem::getElbowSpeed() {
    return elbowPivotMotor.GetAbsoluteVelocity();
}

//super epic PID stuff

void ElbowSubsystem::setElbowTarget() {
    elbowPivotMotor.setReference(getElbowTargetAngle(), Motor::controlType::position);
}

 //gets the gripper pivot angle
double ElbowSubsystem::getGripperPivotAngle() {
    return gripperPivotMotor.GetAbsolutePosition();
}

double ElbowSubsystem::getTargetGripperPivotAngle() {
    return elbowTargetAngle;
}

double ElbowSubsystem::getGripperPivotSpeed()
{
    return gripperPivotMotor.GetAbsoluteVelocity();
}

bool ElbowSubsystem::getGripperPivotState() {
     return gripperPivotState;
}