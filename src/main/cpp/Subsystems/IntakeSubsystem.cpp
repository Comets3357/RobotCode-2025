#include "Subsystems/IntakeSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"

IntakeSubsystem::IntakeSubsystem() {
    AlgaeDeploy.setPID(0.1, 0, 0);
    AlgaeDeploy.setAbsolutePositionConversionFacotr(360);
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

void IntakeSubsystem::SetAngle(units::degree_t angle) {
    AlgaeDeploy.SetRelativePosition(angle.value());
}

double IntakeSubsystem::GetAngle() {
    return AlgaeDeploy.GetAbsolutePosition();
}