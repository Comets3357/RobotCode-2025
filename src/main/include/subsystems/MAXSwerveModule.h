
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
// #include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include "wrapperclasses/SparkMaxMotor.h"
#include "wrapperclasses/SparkFlexMotor.h"
#include "Constants.h"
using namespace rev::spark;

class MAXSwerveModule
{
public:
    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    MAXSwerveModule(int driveCANId, int turningCANId,
                    double chassisAngularOffset);

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    frc::SwerveModuleState GetState();

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    frc::SwerveModulePosition GetPosition();

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    void SetDesiredState(const frc::SwerveModuleState &state);

    /**
     * Zeroes all the SwerveModule encoders.
     */
    void ResetEncoders();

private:
    SparkFlexMotor m_drivingSpark;
    SparkMaxMotor m_turningSpark;

    double m_chassisAngularOffset = 0;
    frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0},
                                          frc::Rotation2d()};
};
