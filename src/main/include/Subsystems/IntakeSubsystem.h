#pragma once
#include <frc2/command/SubsystemBase.h>
#include "wrapperclasses/SparkMaxMotor.h"

class IntakeSubsystem : public frc2::SubsystemBase { 
public: 
    IntakeSubsystem();
    void Intake();
    void Eject();
    void Stop();
private:
    SparkMaxMotor AlgaeIntake{21};
    SparkMaxMotor AlgaeDeploy{22};
};

