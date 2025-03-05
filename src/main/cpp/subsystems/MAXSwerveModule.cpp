#include "subsystems/MAXSwerveModule.h"

#include <frc/geometry/Rotation2d.h>

using namespace rev::spark;

MAXSwerveModule::MAXSwerveModule(const int drivingCANId, const int turningCANId,
                                 const double chassisAngularOffset)
    : m_drivingSpark(drivingCANId),
      m_turningSpark(turningCANId)
{

    double drivingFactor = ModuleConstants::kWheelDiameter.value() *
                           std::numbers::pi /
                           ModuleConstants::kDrivingMotorReduction;

    // configurations for the driving spark max //
    m_drivingSpark.setPID(0.05, 0, 0, 0.18);                                  // 0.0025
    m_drivingSpark.setRelativeVelocityConversionFactor(drivingFactor / 60.0); // meters per second
    m_drivingSpark.setRelativePositionConversionFactor(drivingFactor);        // meters

    m_drivingSpark.SetSmartCurrentLimit(50);
    m_drivingSpark.setFeedbackSensor(Motor::encoderType::relative);
    m_drivingSpark.setInverted(true);

    // configurations for the turning spark max //

    double turningFactor = 2 * std::numbers::pi;

    m_turningSpark.SetSmartCurrentLimit(50);
    m_turningSpark.setInverted(false);
    m_turningSpark.setAbsolutePositionConversionFactor(turningFactor);        // radians
    m_turningSpark.setAbsoluteVelocityConversionFactor(turningFactor / 60.0); // radians per second
    m_turningSpark.setFeedbackSensor(Motor::encoderType::absolute);
    m_turningSpark.setPID(5, 0, 0);
    m_turningSpark.setOutputRange(-1, 1);
    m_turningSpark.setPositionWrappingEnabled(true);
    m_turningSpark.setPositionWrappingMaxRange(0, turningFactor);
    m_turningSpark.setAbsoluteEncoderInverted(true);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle =
        frc::Rotation2d(units::radian_t{m_turningSpark.GetAbsolutePosition()});
    m_drivingSpark.SetRelativePosition(0);

    m_drivingSpark.configure();
    m_turningSpark.configure();
}

frc::SwerveModuleState MAXSwerveModule::GetState()
{
    return {units::meters_per_second_t{m_drivingSpark.GetRelativeVelocity()},
            units::radian_t{m_turningSpark.GetAbsolutePosition() -
                            m_chassisAngularOffset}};
}

frc::SwerveModulePosition MAXSwerveModule::GetPosition()
{
    return {units::meter_t{m_drivingSpark.GetRelativePosition()},
            units::radian_t{m_turningSpark.GetAbsolutePosition() -
                            m_chassisAngularOffset}};
}

void MAXSwerveModule::SetDesiredState(
    const frc::SwerveModuleState &desiredState)
{
    // Apply chassis angular offset to the desired state.
    frc::SwerveModuleState correctedDesiredState{};
    correctedDesiredState.speed = desiredState.speed;
    correctedDesiredState.angle =
        desiredState.angle +
        frc::Rotation2d(units::radian_t{m_chassisAngularOffset});

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.Optimize(
        frc::Rotation2d(units::radian_t{m_turningSpark.GetAbsolutePosition()}));

    m_drivingSpark.setReference(
        (double)correctedDesiredState.speed, Motor::controlType::velocity);
    m_turningSpark.setReference(
        correctedDesiredState.angle.Radians().value(),
        Motor::controlType::position);

    m_desiredState = desiredState;
}

void MAXSwerveModule::ResetEncoders() { m_drivingSpark.SetRelativePosition(0); }