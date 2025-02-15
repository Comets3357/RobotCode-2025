#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "wrapperclasses/Motor.h"
#include "wrapperclasses/SparkMaxMotor.h"
#include <frc/smartdashboard/SmartDashboard.h>

class MotorTest : public frc2::SubsystemBase
{
    public:
        MotorTest();

        void setSpeed(double speed);
    private://  rev::SparkMaxLimitSwitch ElevatorMax = MainElevatorMotor.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
        SparkMaxMotor WristMotor{17};
};
        