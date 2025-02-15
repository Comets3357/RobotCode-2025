#include "Commands/MotorTestCommand.h"
#include "frc2/command/Commands.h"

frc2::CommandPtr TurnOn(MotorTest* motor) {
    return frc2::cmd::RunOnce([motor] {
        motor->setSpeed(0.2);
    });
}

frc2::CommandPtr TurnOff(MotorTest* motor) {
    return frc2::cmd::RunOnce([motor] {
        motor->setSpeed(0);
    });
}