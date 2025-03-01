#include "Commands/IntakeCommands.h"
#include "frc2/command/Commands.h"

//Command to intake algae @param intake Intake Subsystem
frc2::CommandPtr IntakeAlgae(IntakeSubsystem* intake) {
    return frc2::cmd::RunOnce([intake] {
        intake->Intake(-0.3);
        intake->SetAngle(175_deg);
    });
}

//Command to eject algae
frc2::CommandPtr DeployAlgae(IntakeSubsystem* intake) {
    return frc2::cmd::RunOnce([intake] {
        intake->Eject(-0.3);
        intake->SetAngle(175_deg);
    });
}

//Command to stop algae-related motors
frc2::CommandPtr StopIntake(IntakeSubsystem* intake) {
    return frc2::cmd::RunOnce([intake] {
        intake->Stop();
        intake->SetAngle(175_deg);
    });
}

frc2::CommandPtr StopDeploy(IntakeSubsystem* intake) {
    return frc2::cmd::RunOnce([intake] {
        intake->Stop();
        intake->SetAngle(100_deg);
    });
}