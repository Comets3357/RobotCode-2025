#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "wrapperclasses/Motor.h"
#include "wrapperclasses/SparkMaxMotor.h"

class ElevatorSubsystem : public frc2::SubsystemBase
{
    public:
        ElevatorSubsystem(ElevatorSubsystem* elevator);
        void ElevatorExtend();
        bool ElevatorMax();
        void ElevatorRetract();
        void ElevatorStop();
    private:
        //  rev::SparkMaxLimitSwitch ElevatorMax = MainElevatorMotor.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
        SparkMaxMotor MainElevatorMotor{11};
        SparkMaxMotor FollowElevatorMotor{12};

};
