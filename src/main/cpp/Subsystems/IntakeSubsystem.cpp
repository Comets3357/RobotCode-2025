#include "subsystems/IntakeSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"

IntakeSubsystem::IntakeSubsystem() {
    AlgaeDeploy.setPID(0.006, 0, 0); //PID
    AlgaeDeploy.SetSmartCurrentLimit(30);
    AlgaeDeploy.setFeedbackSensor(Motor::encoderType::absolute);
    AlgaeDeploy.setAbsolutePositionConversionFactor(360);
    AlgaeDeploy.setInverted(true);
    AlgaeDeploy.setForwardSoftLimit(190);
    AlgaeDeploy.setReverseSoftLimit(80);
    AlgaeDeploy.configure();
}

void IntakeSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Algae angle", GetAngle());
    SetAngle(100_deg); 
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

//setters
void IntakeSubsystem::SetAngle(units::degree_t angle) {
    AlgaeDeploy.setReference(angle.value(), Motor::controlType::position);
}

//getters
double IntakeSubsystem::GetAngle() {
    return AlgaeDeploy.GetAbsolutePosition();
}