#include "subsystems/ElbowSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "wrapperclasses/SparkMaxMotor.h"
#include "grpl/CanBridge.h"
#include "Robot.h"
#include <cmath>

ElbowSubsystem::ElbowSubsystem() {

    if (isCompBot == true) {
        elbowMotor = new SparkFlexMotor{elbowPivotID};
    } else {
        elbowMotor = new SparkMaxMotor{elbowPivotID};
    }

    if (isCompBot == false) {
         //elbow stuff
        elbowMotor->SetSmartCurrentLimit(60);
        elbowMotor->setRelativePositionConversionFactor(10);
        elbowMotor->setRelativeVelocityConversionFactor(0.167);
        elbowMotor->enableForwardSoftLimit(true);
        elbowMotor->enableReverseSoftLimit(true);
        elbowMotor->setForwardSoftLimit(117);
        elbowMotor->setReverseSoftLimit(40); 

        //limiting so we dont give josh a bad time if we break it
        elbowMotor->setMaxOutput(0.5);
        elbowMotor->setMinOutput(-0.5);

        elbowMotor->setFeedbackSensor(Motor::encoderType::relative);
        elbowMotor->setPID(elbowP, elbowI, elbowD);

        //gripper stuff
        wristMotor.SetSmartCurrentLimit(20);
        wristMotor.setRelativeVelocityConversionFactor(0.06 /* goofy ahh value I dont know*/);
        wristMotor.setRelativePositionConversionFactor(3.6 /* 360 degrees / 25 / 4 for ratios*/);
        wristMotor.setAbsolutePositionConversionFactor(360 /* Degrees */);
        wristMotor.setPositionWrappingEnabled(true);
        wristMotor.setPositionWrappingMaxRange(0, 360);

        wristMotor.setFeedbackSensor(Motor::encoderType::relative);
        wristMotor.setPID(wristP, wristI, wristD);


        elbowMotor->configure();
        wristMotor.configure();

        rollerMotor.SetSmartCurrentLimit(60);  
    }
    if (isCompBot == true) {
        elbowMotor->SetSmartCurrentLimit(60);
        elbowMotor->setAbsoluteVelocityConversionFactor(6);
        elbowMotor->setAbsolutePositionConversionFactor(360 /*degrees*/);

        elbowMotor->enableForwardSoftLimit(true);
        elbowMotor->enableReverseSoftLimit(true);
        elbowMotor->setForwardSoftLimit(305);
        elbowMotor->setReverseSoftLimit(60); 

        //limiting so we dont give josh a bad time if we break it
        elbowMotor->setMaxOutput(0.5);
        elbowMotor->setMinOutput(-0.5);

        elbowMotor->setFeedbackSensor(Motor::encoderType::absolute);
        elbowMotor->setPID(elbowP, elbowI, elbowD);

        //gripper stuff
        wristMotor.SetSmartCurrentLimit(40);
        wristMotor.setAbsoluteVelocityConversionFactor(6);
        wristMotor.setAbsolutePositionConversionFactor(360 /* Degrees */);
        wristMotor.setAbsoluteEncoderInverted(true);
        wristMotor.setPositionWrappingEnabled(true);
        wristMotor.setPositionWrappingMaxRange(0, 360);

        wristMotor.setFeedbackSensor(Motor::encoderType::absolute);
        wristMotor.setPID(wristP, wristI, wristD);


        elbowMotor->configure();
        wristMotor.configure();

        rollerMotor.SetSmartCurrentLimit(20);  
    }    
}

//setters

//sets the elbow angle (360 degrees)
void ElbowSubsystem::setElbowAngle(int angle) {
    elbowMotor->setReference(angle, Motor::controlType::position);
}

//sets the speed in percent
void ElbowSubsystem::setElbowSpeed(double speed) {
    elbowMotor->SetPercent(speed);
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
    return elbowMotor->GetAbsolutePosition();
}

//gets the target angle
double ElbowSubsystem::getElbowTargetAngle() {
    return elbowTargetAngle;
}

//gets the velocity of the elbow pivot motor
double ElbowSubsystem::getElbowSpeed() {
    return elbowMotor->GetAbsoluteVelocity();
}

//super epic PID stuff

void ElbowSubsystem::setElbowTarget() {
    //TODO PUT THIS BACK
   // elbowMotor->setReference(getElbowTargetAngle(), Motor::controlType::position, -0.12);
   elbowMotor->setReference(getElbowTargetAngle(), Motor::controlType::position);
}

void ElbowSubsystem::setWristTarget() {
    wristMotor.setReference(getTargetWristAngle(), Motor::controlType::position);
}

//erm what the sideroni
void ElbowSubsystem::setSideOne(double value) {
    sideOne = value;
}
void ElbowSubsystem::setSideTwo(double value) {
    sideTwo = value;
}

//WAITWAITWAITWAITWAIT what side are we getting
double ElbowSubsystem::getSideOne() {
    return sideOne;
}
double ElbowSubsystem::getSideTwo() {
    return sideTwo;
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

double ElbowSubsystem::getRollerCurrent() {
    return rollerMotor.GetOutputCurrent();
}

void ElbowSubsystem::WristRotate()
{
    if (wristMotor.GetAbsolutePosition() < 145 && wristMotor.GetAbsolutePosition() > 35)
    {
        flip = false;
    } else {
        flip = true;
    }

    if (flip == false) {
        flip = true;
        wristMotor.setReference(0, Motor::controlType::position);
    } else {
        flip = false;
         wristMotor.setReference(180, Motor::controlType::position);
    }
}

void ElbowSubsystem::detectionRotate() {
    if (wristMotor.GetAbsolutePosition() < 2 && wristMotor.GetAbsolutePosition() > 358 ) {
        flip = false;
    } else {
        flip = true;
    }

    if (flip == false) {
        flip = true;
        wristMotor.setReference(90, Motor::controlType::position);
    } else {
        flip = false;
         wristMotor.setReference(270, Motor::controlType::position);
    }
}

double ElbowSubsystem::getDistanceMeasurement() {
    if (ElbowSubsystem::getLaserCANMeasurement().has_value()) {
        return getLaserCANMeasurement().value().distance_mm;
    } else {
        return 1000;
    }

}

std::optional<grpl::LaserCanMeasurement> ElbowSubsystem::getLaserCANMeasurement() {
    return laserCANMeasurement;
}


bool ElbowSubsystem::isGamePieceDetected() {

    double tempDistanceMeasurement;

    if (ElbowSubsystem::getLaserCANMeasurement().has_value()) {
        tempDistanceMeasurement = getLaserCANMeasurement().value().distance_mm;
    } else {
        tempDistanceMeasurement = 1000;
    }

    if (tempDistanceMeasurement < 90) {
        return true;
    } else {
        return false;
    }
}

bool ElbowSubsystem::isAutonWristFlipValid() {
    double sensorToPivot = 66.675;
    double tempDiff = std::abs(ElbowSubsystem::getSideOne() - ElbowSubsystem::getSideTwo());
    double angle = std::atan2(tempDiff, (2 * sensorToPivot));
    angle = angle * (180/3.14159);


    if (angle > 35) {
        return true;
    } else {
        return false;
    }

}

double ElbowSubsystem::arctanAngle(double tempDiff) {
    return std::atan2(tempDiff, (2 * 66.675));
}

void ElbowSubsystem::Periodic() {

    laserCANMeasurement = LaserCAN.get_measurement();


    if (laserCANMeasurement.has_value() && laserCANMeasurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
        frc::SmartDashboard::PutNumber("LaserCAN Measurement", laserCANMeasurement.value().distance_mm);
    }

    frc::SmartDashboard::PutBoolean("Auton Wrist Flip?", ElbowSubsystem::isAutonWristFlipValid());
    frc::SmartDashboard::PutNumber("SideOne", ElbowSubsystem::getSideOne());
    frc::SmartDashboard::PutNumber("SideTwo", ElbowSubsystem::getSideTwo());
    frc::SmartDashboard::PutNumber("auton flip angle", (180/3.14159) * std::atan2(std::abs(sideOne - sideTwo), (2 * 66.675)));
    frc::SmartDashboard::PutNumber("elbow angle", elbowMotor->GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("Wrist Angle", wristMotor.GetAbsolutePosition());
}