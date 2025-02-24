#include "wrapperclasses/SparkMaxMotor.h"
#include "Subsystems/ClimbSubsystem.h"
#include "Robot.h"

ClimbSubsystem::ClimbSubsystem() {

}

ClimbSubsystem::setClimbSpeed(double percent) {
    climbMotor.SetPercent(percent);
}

ClimbSubsystem::getClimbAngle() {
    climbMotor.GetRelativePosition();
}

ClimbSubsystem::getClimbSpeed() {
    climbMotor.GetRelativeVelocity();
}