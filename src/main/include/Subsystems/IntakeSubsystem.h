#pragma once
#include <frc2/command/SubsystemBase.h>
#include "wrapperclasses/SparkMaxMotor.h"
#include <units/angle.h>

class IntakeSubsystem : public frc2::SubsystemBase { 
public: 
    IntakeSubsystem();
    void Periodic() override;
    void Intake();
    void Eject();
    void Stop();
    void Intake(double percent);
    void Eject(double percent);
    void SetAngle(units::degree_t angle);
    double GetAngle();
private:
    SparkMaxMotor AlgaeIntake{22};
    SparkMaxMotor AlgaeDeploy{21};
};

