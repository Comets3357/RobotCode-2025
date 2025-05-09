#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "wrapperclasses/Motor.h"
#include "wrapperclasses/SparkMaxMotor.h"

class ClimbSubsystem : public frc2::SubsystemBase
{
    public:
        ClimbSubsystem();
        void ClimbMax();
        void ClimbRetract();
        void ClimbStop();
        void ClimbSetPercent(double percent);
        void ClimbSetPosition(double position);
        double GetClimbAbsolutePosition();
        bool isRunning();

        void Periodic() override;
    private:
        //  rev::SparkMaxLimitSwitch ElevatorMax = MainElevatorMotor.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
        SparkMaxMotor climbMotor{31};
};