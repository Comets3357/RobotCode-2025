// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #pragma once

// // #include <frc/ADIS16470_IMU.h>
// #include <frc2/command/SubsystemBase.h>
// #include <frc/controller/PIDController.h>
// #include "wrapperclasses/SparkMaxMotor.h"
// #include "wrapperclasses/SparkFlexMotor.h"
// #include <frc2/command/CommandPtr.h>
// #include <frc2/command/Command.h>
// #include "Constants.h"
// #include "MAXSwerveModule.h"

// class ClimbSubsystem : public frc2::SubsystemBase
// {
// public:
//     ClimbSubsystem();

//     double climbMotorID = 31;
//     void setClimbSpeed(double percent);
//     double getClimbSpeed();
//     double getClimbAngle();

//     void Periodic() override;

// private:
//     // Components (e.g. motor controllers and sensors) should generally be
//     // declared private and exposed only through public methods.
//     SparkMaxMotor climbMotor{31};
// };