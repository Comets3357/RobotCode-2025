#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

class ElevatorSubsystem : public frc2::SubsystemBase
{
    public:
        ElevatorSubsystem(ElevatorSubsystem* elevator);
        void ElevatorExtend();
        bool ElevatorMax();
        void ElevatorStop();
    private:
         rev::CANSparkMax MainElevatorMotor {11, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
         rev::CANSparkMax FollowElevatorMotor {12, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
         rev::SparkMaxLimitSwitch ElevatorMax = MainElevatorMotor.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
         rev::SparkRelativeEncoder encoder = MainElevatorMotor.GetEncoder();
};
