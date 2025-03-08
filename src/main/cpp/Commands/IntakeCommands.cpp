#include "Commands/IntakeCommands.h"
#include "frc2/command/Commands.h"

//Command to m_intake algae @param m_intake Intake Subsystem
frc2::CommandPtr IntakeAlgae(IntakeSubsystem* m_intake) {
    return frc2::cmd::RunOnce([m_intake] {
        m_intake->Intake(-0.3);
        m_intake->SetAngle(175_deg);
    });
}

//Command to eject algae
frc2::CommandPtr DeployAlgae(IntakeSubsystem* m_intake) {
    return frc2::cmd::RunOnce([m_intake] {
        m_intake->Eject(-0.3);
        m_intake->SetAngle(175_deg);
    });
}

//Command to stop algae-related motors
frc2::CommandPtr StopIntake(IntakeSubsystem* intake) {
    return frc2::cmd::RunOnce([intake] {
        intake->Stop();
        intake->SetAngle(175_deg);
    });
}

frc2::CommandPtr StopDeploy(IntakeSubsystem* m_intake) {
    return frc2::cmd::RunOnce([m_intake] {
        m_intake->Stop();
        m_intake->SetAngle(100_deg);
    });
}