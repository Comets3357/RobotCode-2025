#include "Commands/IntakeCommands.h"
#include "frc2/command/Commands.h"

//Command to intake algae @param intake Intake Subsystem
frc2::CommandPtr IntakeAlgae(IntakeSubsystem* intake) {return frc2::cmd::Run([intake] {intake->Intake();});}

//Command to eject algae
frc2::CommandPtr DeployAlgae(IntakeSubsystem* intake) {return frc2::cmd::Run([intake] {intake->Eject();});}

//Command to stop algae-related motors
frc2::CommandPtr StopIntake(IntakeSubsystem* intake) {return frc2::cmd::Run([intake] {intake->Stop();});}

frc2::CommandPtr ChangeAngle(IntakeSubsystem* intake, units::degree_t angle) {
    return frc2::cmd::RunOnce([intake, angle] {intake->SetAngle(angle);});
}