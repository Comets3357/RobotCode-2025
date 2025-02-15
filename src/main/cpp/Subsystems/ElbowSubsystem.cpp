#include "subsystems/ElbowSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "wrapperclasses/SparkMaxMotor.h"
#include "Subsystems/ElbowSubsystem.h"
#include "grpl/CanBridge.h"
#include "Robot.h"

ElbowSubsystem::ElbowSubsystem() {

    //elbow stuff
    elbowPivotMotor.setRelativeVelocityConversionFactor(4.5);
    elbowPivotMotor.enableForwardSoftLimit(true);
    elbowPivotMotor.enableReverseSoftLimit(true);
    elbowPivotMotor.setForwardSoftLimit(-5);
    elbowPivotMotor.setReverseSoftLimit(-40);  
    elbowPivotMotor.SetSmartCurrentLimit(8);  

    //gripper stuff
    gripperPivotMotor.setRelativeVelocityConversionFactor(0.06 /* goofy ahh value I dont know*/);
    gripperPivotMotor.setRelativePositionConversionFactor(3.6 /* 360 degrees / 25 / 4 for ratios*/);
    elbowPivotMotor.SetSmartCurrentLimit(20);  

    //limiting so we dont give josh a bad time if we break it
    elbowPivotMotor.setMaxOutput(0.1);

    gripperPivotMotor.setAbsolutePositionConversionFactor(360 /* Degrees */);
    elbowPivotMotor.SetSmartCurrentLimit(20);

    elbowPivotMotor.setFeedbackSensor(Motor::encoderType::absolute);
    gripperPivotMotor.setFeedbackSensor(Motor::encoderType::relative);


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
    //TODO PUT THIS BACK
    //elbowPivotMotor.setReference(getElbowTargetAngle(), Motor::controlType::position);
}

void ElbowSubsystem::setGripperTarget() {
    //TODO PUT THIS BACK
    //gripperPivotMotor.setReference(getGripperTargetAngle(), Motor::controlType::position);
}

 //gets the gripper pivot angle
double ElbowSubsystem::getGripperPivotAngle() {
    return gripperPivotMotor.GetAbsolutePosition();
}

double ElbowSubsystem::getGripperTargetAngle() {
    return elbowTargetAngle;
}

double ElbowSubsystem::getGripperPivotSpeed()
{
    return gripperPivotMotor.GetAbsoluteVelocity();
}

bool ElbowSubsystem::getGripperPivotState() {
     return gripperPivotState;
}

std::optional<grpl::LaserCanMeasurement> ElbowSubsystem::getHorizontalDistanceMeasurement() {
    return horizMeasurement;
}

std::optional<grpl::LaserCanMeasurement> ElbowSubsystem::getVerticalDistanceMeasurement() {
    return vertMeasurement;
}

bool ElbowSubsystem::isGamePieceDetected() {

    double tempVertMeasurement;
    double tempHorizMeasurement;

    if (ElbowSubsystem::getHorizontalDistanceMeasurement().has_value()) {
        tempHorizMeasurement = getHorizontalDistanceMeasurement().value().distance_mm;
    }
    if (ElbowSubsystem::getVerticalDistanceMeasurement().has_value()) {
        tempVertMeasurement = getVerticalDistanceMeasurement().value().distance_mm;
    }

    if (tempVertMeasurement || tempHorizMeasurement < 50) {
        return true;
    } else {
        return false;
    }
}

void ElbowSubsystem::Periodic() {
    vertMeasurement = LaserCanVertical.get_measurement();
    horizMeasurement = LaserCanHorizontal.get_measurement();


    if (vertMeasurement.has_value() && vertMeasurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
        frc::SmartDashboard::PutNumber("Vertical LaserCAN Measurement", vertMeasurement.value().distance_mm);
    }
    if (horizMeasurement.has_value() && horizMeasurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
        frc::SmartDashboard::PutNumber("Horizontal LaserCAN Measurement", horizMeasurement.value().distance_mm);
    }
}