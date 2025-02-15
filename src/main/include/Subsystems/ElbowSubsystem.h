#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include "wrapperclasses/SparkMaxMotor.h"
#include "grpl/CanBridge.h"
#include "grpl/LaserCan.h"

class ElbowSubsystem : public frc2::SubsystemBase {
public:

    ElbowSubsystem();

    //setting states for the gripper
    enum gripperStates{INTAKE, OUTTAKE, IDLE}
    gripperState{gripperStates::IDLE};

    //constants
    int elbowPivotID = 16;
    int gripperPivotID = 17;
    int armGripperID = 18;

    
    //epic PID system for the elbow and gripper, dont change these values unless you know what you are doing :)
    const double elbowP = 0.04;
    const double elbowI = 0;
    const double elbowD = 0;
    double elbowTargetAngle = -10;

    const double gripperP = 0.02;
    const double gripperI = 0;
    const double gripperD = 0;
    double gripperTargetAngle = 0;
    bool gripperPivotState = false;

    //initalizing measurement vehicles
    std::optional<grpl::LaserCanMeasurement> vertMeasurement;
    std::optional<grpl::LaserCanMeasurement> horizMeasurement;

    //
    //setters
    //

    //elbow setters
    
    //gets the target Elbow Angle.
    void setTargetElbowAngle(int angle);

    //Sets which encoder to use for the PID calculations for the elbow (DONT CHANGE THIS.)
    void setElbowTarget();

    //sets the elbow speed for testing purposes.
    void setElbowSpeed(double speed);

    //gripper pivot setters

    //sets the state of the gripper pivot to vertical or horizontal (TERRIBLE EXPLAINATION CHANGE THIS LATER)
    void setGripperPivotState(bool state);

    //toggles the gripper pivot state
    void toggleGripperPivotState();

    //sets the target gripper pivot angle.
    void setGripperPivotAngle(double angle);
    
    //sets the gripper pivot speed
    void setGripperPivotSpeed(double speed);

    //gripper setters

    //sets the gripper roller speed
    void setGripperSpeed(double speed);

    //sets the gripper state between three enumerables: IDLE, INTAKE, OUTTAKE.
    void setGripperState(gripperStates state);

    void setGripperTarget();
    
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
    double getGripperPivotAngle();

    //gets the target gripper pivot angle
    double getGripperTargetAngle();

    //gets the gripper pivot speed in 
    double getGripperPivotSpeed();

    //gets the gripper pivot state in Vertical to horizontal or whatever this might be useless
    bool getGripperPivotState();

    //gripper roller getters
    gripperStates getGripperState();

    //is game piece detected via the distance sensor
    bool isGamePieceDetected();

    //getting the measurement of the horizontal and vertical distance measurements
    std::optional<grpl::LaserCanMeasurement>  getHorizontalDistanceMeasurement();
    std::optional<grpl::LaserCanMeasurement>  getVerticalDistanceMeasurement();

    void Periodic() override;

private:
    SparkMaxMotor elbowPivotMotor{elbowPivotID}; 
    SparkMaxMotor gripperPivotMotor{gripperPivotID};
    SparkMaxMotor gripperMotor{armGripperID};

    grpl::LaserCan LaserCanVertical{19};
    grpl::LaserCan LaserCanHorizontal{20};


};