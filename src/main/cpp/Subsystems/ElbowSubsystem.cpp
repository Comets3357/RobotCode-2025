#include "subsystems/ElbowSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "wrapperclasses/SparkMaxMotor.h"
//#include "grpl/CanBridge.h"
#include "Robot.h"

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
        elbowMotor->setMaxOutput(1);
        elbowMotor->setMinOutput(-1);

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
        wristMotor.setAbsoluteEncoderInverted(false);
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

double ElbowSubsystem::getRollerCurrent() {
    return rollerMotor.GetOutputCurrent();
}

void ElbowSubsystem::WristRotate()
{
    if (wristMotor.GetAbsolutePosition() < 145 && wristMotor.GetAbsolutePosition() > 35)
    {
        flip = false;
    }
    else
    {
        flip = true;
    }

    if(flip == false)
    {
        flip = true;
        
        // wristMotor.setReference(0, Motor::controlType::position);
        wristMotor.setReference(0, Motor::controlType::position);
        // wristMotor.setReference(270, Motor::controlType::position);
    }
    else
    {
        flip = false;

         wristMotor.setReference(180, Motor::controlType::position);
        // wristMotor.setReference(180, Motor::controlType::position);
        // wristMotor.setReference(90, Motor::controlType::position);
    }
}

// std::optional<grpl::LaserCanMeasurement> ElbowSubsystem::getHorizontalDistanceMeasurement() {
//     return horizMeasurement;
// }

// std::optional<grpl::LaserCanMeasurement> ElbowSubsystem::getVerticalDistanceMeasurement() {
//     return vertMeasurement;
// }

bool ElbowSubsystem::isGamePieceDetected() {

    // double tempVertMeasurement;
    // double tempHorizMeasurement;

    // if (ElbowSubsystem::getHorizontalDistanceMeasurement().has_value()) {
    //     tempHorizMeasurement = getHorizontalDistanceMeasurement().value().distance_mm;
    // }
    // if (ElbowSubsystem::getVerticalDistanceMeasurement().has_value()) {
    //     tempVertMeasurement = getVerticalDistanceMeasurement().value().distance_mm;
    // }

    // if (tempVertMeasurement || tempHorizMeasurement < 50) {
    //     return true;
    // } else {
    //     return false;
    // }
    return false; 
}

void ElbowSubsystem::Periodic() {
    // vertMeasurement = LaserCanVertical.get_measurement();
    // horizMeasurement = LaserCanHorizontal.get_measurement();


    // if (vertMeasurement.has_value() && vertMeasurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
    //     frc::SmartDashboard::PutNumber("Vertical LaserCAN Measurement", vertMeasurement.value().distance_mm);
    // }
    // if (horizMeasurement.has_value() && horizMeasurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
    //     frc::SmartDashboard::PutNumber("Horizontal LaserCAN Measurement", horizMeasurement.value().distance_mm);
    // }

    frc::SmartDashboard::PutNumber("elbow angle", elbowMotor->GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("Wrist Angle", wristMotor.GetAbsolutePosition());
}