#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "wrapperclasses/Motor.h"
#include "wrapperclasses/SparkMaxMotor.h"
#include <frc/smartdashboard/SmartDashboard.h>

class ElevatorSubsystem : public frc2::SubsystemBase
{
    public:
        ElevatorSubsystem();

        const double elevatorP = 0.32;
        const double elevatorI = 0;
        const double elevatorD = 3;

        bool CompBotSettings = true;
    
        void setSpeed(double speed);
        void CalculatePID();
        double getRPosition();
        double getAPosition();
        void setPosition(double position);

        void Periodic();
    private:
        //  rev::SparkMaxLimitSwitch ElevatorMax = MainElevatorMotor.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
        SparkMaxMotor MainElevatorMotor{11};
        SparkMaxMotor FollowElevatorMotor{12};  
        
};
        