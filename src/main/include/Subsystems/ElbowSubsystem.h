#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include "wrapperclasses/SparkMaxMotor.h"
#include "wrapperclasses/SparkFlexMotor.h"
#include "grpl/CanBridge.h"
#include "grpl/LaserCan.h"

class ElbowSubsystem : public frc2::SubsystemBase {
public:

    ElbowSubsystem();

    //constants
    int elbowPivotID = 16;
    int gripperPivotID = 17;
    int armGripperID = 18;

    
    //epic PID system for the elbow and gripper, dont change these values unless you know what you are doing :)
    const double elbowP = 0.06 /* Comp Bot -> 0.06; Practice Bot -> 0.01*/;
    const double elbowI = 0;
    const double elbowD = 1.42 /* Comp Bot -> 1.42; Practice Bot -> 0*/;
    double elbowTargetAngle = -10;

    const double wristP = 0.02 /* Practice Bot -> 0.02*/;
    const double wristI = 0;
    const double wristD = 0;
    double targetWristAngle = 0;

    bool isCompBot = true;



    //initalizing measurement vehicles
    std::optional<grpl::LaserCanMeasurement> laserCANMeasurement;

    //
    //setters
    //

    //elbow setters
    
    //gets the target Elbow Angle.
    void setElbowAngle(int angle);

    //Sets which encoder to use for the PID calculations for the elbow (DONT CHANGE THIS.)
    void setElbowTarget();

    //sets the elbow speed for testing purposes.
    void setElbowSpeed(double speed);
    
    //sets the target gripper pivot angle.
    void setWristAngle(double angle);
    
    //sets the gripper pivot speed
    void setWristSpeed(double speed);

    //gripper setters

    //sets the gripper roller speed
    void setRollerSpeed(double speed);

    void setWristTarget();
    
    //
    //getters
    //

    //elbow getters

    //gets the target angle for the elbow pivot.
    double getElbowTargetAngle();

    //gets the actual angle of the elbow pivot.
    double getElbowAngle();

    //gets the speed of the elbow in -1 - 1.
    double getElbowSpeed();

    //gripper pivot getters

    //gets the gripper pivot angle
    double getWristAngle();

    //gets the target gripper pivot angle
    double getTargetWristAngle();

    //gets the gripper pivot speed in 
    double getWristSpeed();

    //gets the current of the rollers
    double getRollerCurrent();
    
    //is game piece detected via the distance sensor
    bool isGamePieceDetected();

    //is flip of wrist necessary for auton
    bool isAutonWristFlipValid(double sideOne, double sideTwo);

    double getDistanceMeasurement();

    //getting the measurement of the horizontal and vertical distance measurements
    std::optional<grpl::LaserCanMeasurement>  getLaserCANMeasurement();
    // std::optional<grpl::LaserCanMeasurement>  getVerticalDistanceMeasurement();

    void Periodic() override;
private:

    Motor* elbowMotor = nullptr;
    SparkMaxMotor wristMotor{gripperPivotID};
    SparkMaxMotor rollerMotor{armGripperID};

    grpl::LaserCan LaserCAN{19};


};