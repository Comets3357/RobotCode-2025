#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <redux/canand/CanandEventLoop.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include "Constants.h"

using namespace DriveConstants;
using namespace pathplanner;


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


     RobotConfig config = RobotConfig::fromGUISettings();

    // Configure the AutoBuilder last
    AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ m_odometry.ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ DriveFromChassisSpeeds(speeds, true); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(4.5, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

void DriveSubsystem::DriveFromChassisSpeeds(frc::ChassisSpeeds speed, bool fieldRelative)
{
    Drive(speed.vx, speed.vy, speed.omega, fieldRelative); 
}

frc::Rotation2d DriveSubsystem::GetGyroHeading()
{

    return m_gyro.GetRotation2d();
}

double DriveSubsystem::GetChassisSpeed()
{
    return (double)m_frontLeft.GetState().speed; 
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
void DriveSubsystem::ZeroHeading(frc::Pose2d degree) {
    m_gyro.SetYaw(degree.Rotation().Degrees(), 50_ms); 
}

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