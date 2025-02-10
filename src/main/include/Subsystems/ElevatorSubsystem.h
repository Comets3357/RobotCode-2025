#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "wrapperclasses/Motor.h"
#include "wrapperclasses/SparkMaxMotor.h"

class ElevatorSubsystem : public frc2::SubsystemBase
{
    public:
        ElevatorSubsystem();

        const double elevatorP = 1;
        const double elevatorI = 0;
        const double elevatorD = 0;

        void setSpeed(int speed);
        void CalculatePID();
        
         void getPosition(double TargetPosition);
         void ElevatorExtend();
         void ElevatorMax();
         void ElevatorRetract();
         void ElevatorStop();
         void setPosition(double position);
    private:
        //  rev::SparkMaxLimitSwitch ElevatorMax = MainElevatorMotor.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
        SparkMaxMotor MainElevatorMotor{11};
        SparkMaxMotor FollowElevatorMotor{12};

};
        