#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>

class ElbowSubsystem : public frc2::SubsystemBase {
public:

    ElbowSubsystem();
    
    //epic PID system for the elbow, dont change these values unless you know what you are doing :)
    const double P = 1;
    const double I = 0;
    const double D = 0;
    
}