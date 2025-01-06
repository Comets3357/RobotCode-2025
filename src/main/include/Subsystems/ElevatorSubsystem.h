#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
//#include <rev/CANSparkMax.h>

class ElevatorSubsystem : public frc2::SubsystemBase
{
    public:
        ElevatorSubsystem(ElevatorSubsystem* elevator);
        void ElevatorExtend();
        bool ElevatorMax();
        void ElevatorStop();
    private:
};
