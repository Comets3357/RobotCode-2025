#include "subsystems/ElbowSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "wrapperclasses/SparkMaxMotor.h"
#include "Subsystems/ElbowSubsystem.h"
#include "grpl/CanBridge.h"
#include "Robot.h"

ElbowSubsystem::ElbowSubsystem() {

    //elbow stuff
    elbowMotor.SetSmartCurrentLimit(40);
    elbowMotor.setRelativePositionConversionFactor(10);
    elbowMotor.setRelativeVelocityConversionFactor(0.167);
    elbowMotor.enableForwardSoftLimit(true);
    elbowMotor.enableReverseSoftLimit(true);
    elbowMotor.setForwardSoftLimit(117);
    elbowMotor.setReverseSoftLimit(40); 

    //limiting so we dont give josh a bad time if we break it
    elbowMotor.setMaxOutput(0.5);
    elbowMotor.setMinOutput(-0.5);

    elbowMotor.setFeedbackSensor(Motor::encoderType::relative);
    elbowMotor.setPID(elbowP, elbowI, elbowD);

    //gripper stuff
    wristMotor.SetSmartCurrentLimit(20);
    wristMotor.setRelativeVelocityConversionFactor(0.06 /* goofy ahh value I dont know*/);
    wristMotor.setRelativePositionConversionFactor(3.6 /* 360 degrees / 25 / 4 for ratios*/);
    wristMotor.setAbsolutePositionConversionFactor(360 /* Degrees */);

    wristMotor.setFeedbackSensor(Motor::encoderType::relative);
    wristMotor.setPID(wristP, wristI, wristD);


    elbowMotor.configure();
    wristMotor.configure();

    rollerMotor.SetSmartCurrentLimit(40);  
}

//setters

//sets the elbow angle (360 degrees)
void ElbowSubsystem::setElbowAngle(int angle) {
    elbowMotor.setReference(angle, Motor::controlType::position);
}

//sets the speed in percent
void ElbowSubsystem::setElbowSpeed(double speed) {
    elbowMotor.SetPercent(speed);
}

void ElbowSubsystem::setRollerSpeed(double speed) {
    rollerMotor.SetPercent(speed);
}

void ElbowSubsystem::setWristAngle(double angle) {
    wristMotor.setReference(angle, Motor::controlType::position);
}

void ElbowSubsystem::setWristSpeed(double speed) {
    wristMotor.SetPercent(speed);
}

//getters

//gets the elbow angle in absolute position
double ElbowSubsystem::getElbowAngle() {
    return elbowMotor.GetAbsolutePosition();
}

//gets the target angle
double ElbowSubsystem::getElbowTargetAngle() {
    return elbowTargetAngle;
}

//gets the velocity of the elbow pivot motor
double ElbowSubsystem::getElbowSpeed() {
    return elbowMotor.GetAbsoluteVelocity();
}

//super epic PID stuff

void ElbowSubsystem::setElbowTarget() {
    //TODO PUT THIS BACK
    elbowMotor.setReference(getElbowTargetAngle(), Motor::controlType::position, -0.12);
}

void ElbowSubsystem::setWristTarget() {
    //TODO PUT THIS BACK
    wristMotor.setReference(getTargetWristAngle(), Motor::controlType::position);
}

 //gets the gripper pivot angle
double ElbowSubsystem::getWristAngle() {
    return wristMotor.GetAbsolutePosition();
}

double ElbowSubsystem::getTargetWristAngle() {
    return targetWristAngle;
}

double ElbowSubsystem::getWristSpeed() {
    return wristMotor.GetAbsoluteVelocity();
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