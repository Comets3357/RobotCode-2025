#include <frc/smartdashboard/SmartDashboard.h>

#include "Commands/DefaultElbowCommand.h"
#include "Subsystems/ElbowSubsystem.h"
#include <frc2/command/Commands.h>

DefaultElbowCommand::DefaultElbowCommand(ElbowSubsystem *elbowSubsystem, std::function<double()> rightStickSupplier,
 std::function<double()> rightTriggerSupplier)

    : elbowSubsystem{elbowSubsystem} {
    // Register that this command requires the subsystem
    AddRequirements(elbowSubsystem);
    this->rightStick = rightStickSupplier;
    this->rightTrigger = rightTriggerSupplier;
}

void DefaultElbowCommand::Execute() {


    // elbowSubsystem->setElbowSpeed(rightStick() * 0.1);
    
    // if (rightStick() < -0.1) {
    //    elbowSubsystem->setElbowAngle(elbowSubsystem->getElbowTargetAngle() - 1);
    // }
    // if (rightStick() > 0.1) {
    //     elbowSubsystem->setElbowAngle(elbowSubsystem->getElbowTargetAngle() + 1);
    // }    

    elbowSubsystem->setElbowTarget();
    elbowSubsystem->setWristTarget();

    if (elbowSubsystem->getElbowTargetAngle() > 117) {
        elbowSubsystem->setElbowAngle(117);
    }
    if (elbowSubsystem->getElbowTargetAngle() < 40) {
        elbowSubsystem->setElbowAngle(40);
    }

    //smartdashboard
    frc::SmartDashboard::PutNumber("Right Stick Y Position",rightStick());
    frc::SmartDashboard::PutNumber("Target Elbow Position", elbowSubsystem->getElbowTargetAngle());
    frc::SmartDashboard::PutNumber("Elbow Position", elbowSubsystem->getElbowAngle());

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