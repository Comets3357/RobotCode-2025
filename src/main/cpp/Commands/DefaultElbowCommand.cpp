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
    
    if (rightStick() < -0.1) {
       elbowSubsystem->setTargetElbowAngle(elbowSubsystem->getElbowTargetAngle() - 1);
    }
    if (rightStick() > 0.1) {
        elbowSubsystem->setTargetElbowAngle(elbowSubsystem->getElbowTargetAngle() + 1);
    }    

    elbowSubsystem->setElbowTarget();

    if (elbowSubsystem->getElbowTargetAngle() > 130) {
        elbowSubsystem->setTargetElbowAngle(130);
    }
    if (elbowSubsystem->getElbowTargetAngle() < -120) {
        elbowSubsystem->setTargetElbowAngle(-120);
    }

    //gripper roller stuff

    // if (elbowSubsystem->getGripperState() == 0 /*INTTAKE*/) {
    //     elbowSubsystem->setGripperSpeed(0.2);
    // } else if (elbowSubsystem->getGripperState() ==   1 /*OUTTAKE*/) {
    //     elbowSubsystem->setGripperSpeed(-0.2);

    // } else if (elbowSubsystem->getGripperState() ==   2 /*IDLE*/){
    //     elbowSubsystem->setGripperSpeed(0);
    // }

    //gripper pivot stuff
    if (rightTrigger() > 0.5) {
        elbowSubsystem->toggleGripperPivotState();
        if (elbowSubsystem->getGripperPivotState() == true) {
            elbowSubsystem->setGripperPivotAngle(0);
        } else if (elbowSubsystem->getGripperPivotState() == false) {
            elbowSubsystem->setGripperPivotAngle(90);
        }
    }

    //smartdashboard
    frc::SmartDashboard::PutNumber("Right Stick Y Position",rightStick());
    frc::SmartDashboard::PutNumber("Target Elbow Position", elbowSubsystem->getElbowTargetAngle());
    frc::SmartDashboard::PutNumber("Elbow Position", elbowSubsystem->getElbowAngle());

    frc::SmartDashboard::PutNumber("Gripper State", elbowSubsystem->getGripperState());
}

//super mega epic typing
frc2::CommandPtr DefaultElbowCommand::setIdle(ElbowSubsystem* m_elbowSubsystem) {
    return frc2::cmd::RunOnce([m_elbowSubsystem]{m_elbowSubsystem->setGripperState(ElbowSubsystem::gripperStates::IDLE); });
}
frc2::CommandPtr DefaultElbowCommand::setIntake(ElbowSubsystem* m_elbowSubsystem) {
    return frc2::cmd::RunOnce([m_elbowSubsystem]{m_elbowSubsystem->setGripperState(ElbowSubsystem::gripperStates::INTAKE); });
}
frc2::CommandPtr DefaultElbowCommand::setOuttake(ElbowSubsystem* m_elbowSubsystem) {
    return frc2::cmd::RunOnce([m_elbowSubsystem]{m_elbowSubsystem->setGripperState(ElbowSubsystem::gripperStates::OUTTAKE); });
}