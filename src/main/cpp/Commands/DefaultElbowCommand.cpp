

#include "Commands/DefaultElbowCommand.h"
#include "Subsystems/ElbowSubsystem.h"
#include <frc2/command/Commands.h>

DefaultElbowCommand::DefaultElbowCommand(ElbowSubsystem *elbowSubsystem, std::function<double()> rightStickSupplier)
    : elbowSubsystem{elbowSubsystem} {
    // Register that this command requires the subsystem.
    AddRequirements(elbowSubsystem);
    this->rightStick = rightStickSupplier;
}

void DefaultElbowCommand::Execute() {

    //this stuff is kinda self-explainatory lmao
    if (rightStick() == 0) {
       elbowSubsystem->setTargetElbowAngle(elbowSubsystem->getElbowAngle() +2);
    }
    else if (rightStick() == 180) {
        elbowSubsystem->setTargetElbowAngle(elbowSubsystem->getTargetAngle() -2);
    }    


    //TODO add something with calculate PID here or something idk

    //smartdashboard
}