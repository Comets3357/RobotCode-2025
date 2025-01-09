#include "Subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() {

}

void IntakeSubsystem::Intake() {
    AlgaeIntake.SetPercent(1);
}

void IntakeSubsystem::Eject() {
    AlgaeDeploy.SetPercent(1);
}

void IntakeSubsystem::Intake(double percent) {
    AlgaeIntake.SetPercent(percent);
}

void IntakeSubsystem::Eject(double percent) {
    AlgaeDeploy.SetPercent(percent);
}

void IntakeSubsystem::Stop() {
    AlgaeIntake.SetPercent(0);
    AlgaeDeploy.SetPercent(0);
}