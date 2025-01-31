#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <redux/canand/CanandEventLoop.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{
                     0}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}}
{
    // Usage reporting for MAXSwerve template
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
               HALUsageReporting::kRobotDriveSwerve_MaxSwerve);

    redux::canand::EnsureCANLinkServer();
}

frc::Rotation2d DriveSubsystem::GetGyroHeading()
{

    return m_gyro.GetRotation2d();
}

void DriveSubsystem::Periodic()
{
    // Implementation of subsystem periodic method goes here.
    m_odometry.Update(GetGyroHeading(),
                      {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                       m_frontRight.GetPosition(), m_rearRight.GetPosition()});

                       

    frc::SmartDashboard::PutNumber("Gyro Yaw", units::degree_t(m_gyro.GetYaw()).value());
    frc::SmartDashboard::PutNumber("Drive X (m):", m_odometry.GetPose().Translation().X().value());

    
}

void DriveSubsystem::UpdateOdometry() {
  m_poseEstimator.Update(m_gyro.GetRotation2d(),
                         {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                          m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
 
  // Also apply vision measurements. We use 0.3 seconds in the past as an
  // example -- on a real robot, this must be calculated based either on latency
  // or timestamps.

  std::vector<frc::Pose3d> estimatedPoseVector = m_visionSubsystem.getEstimatedGlobalPose(frc::Pose3d{frc::Translation3d(0_m, 0_m, 0_m), frc::Rotation3d(0_rad, 0_rad, 0_rad)});

  if (estimatedPoseVector.size() == 0)
  {

  } else if (estimatedPoseVector.size() == 1)
  {
    m_poseEstimator.AddVisionMeasurement(estimatedPoseVector.at(0).ToPose2d(), frc::Timer::GetFPGATimestamp()); 
  } else if (estimatedPoseVector.size() == 2)
  {
    m_poseEstimator.AddVisionMeasurement(estimatedPoseVector.at(0).ToPose2d(), frc::Timer::GetFPGATimestamp()); 
    m_poseEstimator.AddVisionMeasurement(estimatedPoseVector.at(1).ToPose2d(), frc::Timer::GetFPGATimestamp()); 
  }

}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative)
{

    // Convert the commanded speeds into the correct units for the drivetrain
    units::meters_per_second_t xSpeedDelivered =
        xSpeed.value() * DriveConstants::kMaxSpeed;
    units::meters_per_second_t ySpeedDelivered =
        ySpeed.value() * DriveConstants::kMaxSpeed;
    units::radians_per_second_t rotDelivered =
        rot.value() * DriveConstants::kMaxAngularSpeed;

    auto states = kDriveKinematics.ToSwerveModuleStates(
        fieldRelative
            ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                  xSpeedDelivered, ySpeedDelivered, rotDelivered,
                  GetGyroHeading())
            : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

    kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    m_frontLeft.SetDesiredState(fl);
    m_frontRight.SetDesiredState(fr);
    m_rearLeft.SetDesiredState(bl);
    m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX()
{
    m_frontLeft.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
    m_frontRight.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
    m_rearLeft.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
    m_rearRight.SetDesiredState(
        frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
    kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                           DriveConstants::kMaxSpeed);
    m_frontLeft.SetDesiredState(desiredStates[0]);
    m_frontRight.SetDesiredState(desiredStates[1]);
    m_rearLeft.SetDesiredState(desiredStates[2]);
    m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders()
{
    m_frontLeft.ResetEncoders();
    m_rearLeft.ResetEncoders();
    m_frontRight.ResetEncoders();
    m_rearRight.ResetEncoders();
}

void DriveSubsystem::ZeroHeading() { m_gyro.SetYaw(units::angle::turn_t{0}, 50_ms); }

double DriveSubsystem::GetTurnRate()
{
    return m_gyro.GetAngularVelocityYaw().value(); //-m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
    m_odometry.ResetPosition(
        GetGyroHeading(),
        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
         m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
        pose);
}

frc::ChassisSpeeds DriveSubsystem::GetRobotRelativeSpeeds()
{
    return kDriveKinematics.ToChassisSpeeds({m_frontLeft.GetState(), m_frontRight.GetState(),
                                             m_rearLeft.GetState(), m_rearRight.GetState()});
}