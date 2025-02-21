#include <frc/smartdashboard/SmartDashboard.h>

#include "Commands/DefaultElbowCommand.h"
#include "Subsystems/ElbowSubsystem.h"
#include <frc2/command/Commands.h>

void DefaultElbowCommand::Execute() {
    //smartdashboard

    frc::SmartDashboard::PutBoolean("isGamePieceDetected?", elbowSubsystem->isGamePieceDetected());
}

//super mega epic typing
frc2::CommandPtr DefaultElbowCommand::setWristPos(ElbowSubsystem* m_elbowSubsystem, double position) {
    return frc2::cmd::RunOnce([=]{m_elbowSubsystem->setWristAngle(position); }, {m_elbowSubsystem});
}
frc2::CommandPtr DefaultElbowCommand::setElbowPos(ElbowSubsystem* m_elbowSubsystem, double position) {
    return frc2::cmd::RunOnce([=]{m_elbowSubsystem->setElbowAngle(position); }, {m_elbowSubsystem});
}

//setters for speed
frc2::CommandPtr DefaultElbowCommand::setWristSpeed(ElbowSubsystem* m_elbowSubsystem, double speed) {
    return frc2::cmd::RunOnce([=]{m_elbowSubsystem->setWristSpeed(speed); }, {m_elbowSubsystem});
}
frc2::CommandPtr DefaultElbowCommand::setRollerSpeed(ElbowSubsystem* m_elbowSubsystem, double speed) {
    return frc2::cmd::RunOnce([=]{m_elbowSubsystem->setRollerSpeed(speed); }, {m_elbowSubsystem});
}
frc2::CommandPtr DefaultElbowCommand::setElbowSpeed(ElbowSubsystem* m_elbowSubsystem, double speed) {
    return frc2::cmd::RunOnce([=]{m_elbowSubsystem->setElbowSpeed(speed); }, {m_elbowSubsystem});
}