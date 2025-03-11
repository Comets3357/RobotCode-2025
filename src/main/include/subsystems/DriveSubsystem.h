// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #include <frc/ADIS16470_IMU.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <redux/sensors/Canandgyro.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"
#include "MAXSwerveModule.h"
#include "wrapperclasses/GyroWrapper.h"
#include "subsystems/VisionSubsystem.h"
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <numbers>
#include <frc/DriverStation.h>
#include <frc/Encoder.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angular_velocity.h>

class DriveSubsystem : public frc2::SubsystemBase
{
public:
    DriveSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    // Subsystem methods go here.

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
     * and the linear speeds have no effect on the angular speed.
     *
     * @param xSpeed        Speed of the robot in the x direction
     *                      (forward/backwards).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to
     *                      the field.
     */
    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
               bool fieldRelative);

    void DriveFromChassisSpeeds(frc::ChassisSpeeds speed, bool fieldRelative); 

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    void SetX();

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    void ResetEncoders();

    /**
     * Sets the drive MotorControllers to a power from -1 to 1.
     */
    void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

   double GetChassisSpeed(); 


    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    // units::degree_t GetHeading();

    frc::Rotation2d GetGyroHeading();

    /**
     * Zeroes the heading of the robot.
     */
    void ZeroHeading();

    void ZeroHeading(frc::Pose2d degree); 

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    double GetTurnRate();

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    frc::Pose2d GetPose();

    frc::ChassisSpeeds GetRobotRelativeSpeeds();

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    void ResetOdometry(frc::Pose2d pose);

    frc::SwerveDriveKinematics<4> kDriveKinematics{
        frc::Translation2d{DriveConstants::kWheelBase / 2,
                           DriveConstants::kTrackWidth / 2},
        frc::Translation2d{DriveConstants::kWheelBase / 2,
                           -DriveConstants::kTrackWidth / 2},
        frc::Translation2d{-DriveConstants::kWheelBase / 2,
                           DriveConstants::kTrackWidth / 2},
        frc::Translation2d{-DriveConstants::kWheelBase / 2,
                           -DriveConstants::kTrackWidth / 2}};

    frc::Field2d m_field;

    void PoseEstimation();

private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.

    VisionSubsystem m_visionSubsystem; 

    MAXSwerveModule m_frontLeft;
    MAXSwerveModule m_rearLeft;
    MAXSwerveModule m_frontRight;
    MAXSwerveModule m_rearRight;
    
    frc::Pose2d EstPose1;
    frc::Pose2d EstPose2;

    //redux::sensors::canandgyro::Canandgyro m_gyro{9};
    GyroWrapper m_gyro; 

    public:
    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
      kDriveKinematics,
      frc::Rotation2d{},
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      frc::Pose2d{},
      {0.1, 0.1, 0.1},
      {0.1, 0.1, 100}
      };

    std::vector<photon::EstimatedRobotPose> estimatedPoseVector;
  
};