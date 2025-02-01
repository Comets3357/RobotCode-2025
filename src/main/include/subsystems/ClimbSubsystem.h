#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "wrapperclasses/Motor.h"
#include "wrapperclasses/SparkMaxMotor.h"

class ClimbSubsystem : public frc2::SubsystemBase
{
    public:
        ClimbSubsystem();
        bool ClimbMax();
        void ClimbRetract();
        void ClimbStop();
        void ClimbSet(double percent);
    private:
        //  rev::SparkMaxLimitSwitch ElevatorMax = MainElevatorMotor.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
        SparkMaxMotor ClimbMotor{31};
};