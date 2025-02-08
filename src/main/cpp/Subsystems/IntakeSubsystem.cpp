#include "Subsystems/IntakeSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"

IntakeSubsystem::IntakeSubsystem() {
    AlgaeDeploy.setPID(0.006, 0, 0);
    AlgaeDeploy.SetSmartCurrentLimit(30);
    AlgaeDeploy.setFeedbackSensor(Motor::encoderType::absolute);
    AlgaeDeploy.setAbsolutePositionConversionFactor(360);
    AlgaeDeploy.setInverted(true);
    AlgaeDeploy.setForwardSoftLimit(190);
    AlgaeDeploy.setReverseSoftLimit(80);
    AlgaeDeploy.configure();
}

void IntakeSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("angle", GetAngle());
}


void IntakeSubsystem::Intake() {
    AlgaeIntake.SetPercent(0.1);
}

void IntakeSubsystem::Eject() {
    AlgaeIntake.SetPercent(-0.1);
}

void IntakeSubsystem::Intake(double percent) {
    AlgaeIntake.SetPercent(percent);
}

void IntakeSubsystem::Eject(double percent) {
    AlgaeIntake.SetPercent(-percent);
}

void IntakeSubsystem::Stop() {
    AlgaeIntake.SetPercent(0);
}

void IntakeSubsystem::moveIntake(double percent) {
    AlgaeDeploy.SetPercent(percent);
}

//setters
void IntakeSubsystem::SetAngle(units::degree_t angle) {
    AlgaeDeploy.SetRelativePosition(angle.value());
}

//getters
double IntakeSubsystem::GetAngle() {
    return AlgaeDeploy.GetAbsolutePosition();
}