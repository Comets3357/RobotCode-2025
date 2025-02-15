#pragma once

#include "Subsystems/MotorTest.h"
#include "frc2/command/CommandPtr.h"

frc2::CommandPtr TurnOn(MotorTest* motor);
frc2::CommandPtr TurnOff(MotorTest* motor);