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
    bool halfSpeed = false; 
    bool gyroZero = false; 
    bool isAutoAligning = false; 

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
    frc::Field2d m_fieldNoVision;

    void PoseEstimation();

    void PoseEstimationNoVisionTest();

    void UpdateNonVisionPose();

    void GoToPos(frc::Pose2d targetPos, double max_output = 0.75);

    // void AutoAlignAroundReef();

    bool inRange(frc::Pose2d driverPose, frc::Pose2d pose1, units::meter_t MOE = 0.01_m, units::angle::degree_t MOEangle = 1_deg); 

    double GetDistance(frc::Pose2d target); // in meters
    double GetDistance(frc::Translation2d target); // in meters 
    frc::Pose2d findNearestTarget(bool isLeftSide); 

    bool ArmGoToLeftSide(); 

  

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

    GyroWrapper m_gyro; 

    frc::Field2d m_mirrorField; 
    frc::Field2d NearestTarget;

    void SetPointPositions(); 

    std::vector<frc::Pose2d> leftBluePoses; 
    std::vector<frc::Pose2d> rightBluePoses; 
    std::vector<frc::Pose2d> leftRedPoses; 
    std::vector<frc::Pose2d> rightRedPoses;
    public:
    

    frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
      kDriveKinematics,
      frc::Rotation2d{},
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      frc::Pose2d{},
      {0.1, 0.1, 0.1},
      {0.1, 0.1, 100.0}
      };

    frc::SwerveDrivePoseEstimator<4> m_poseEstimatorNoVision{
      kDriveKinematics,
      frc::Rotation2d{},
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      frc::Pose2d{},
      {0.1, 0.1, 0.1},
      {0.1, 0.1, 100.0}
      };

    std::vector<photon::EstimatedRobotPose> estimatedPoseVector;

    /*
        ALL poses will be such that if you were the human player on that side
    */

    frc::Pose2d BottomLeftRed{}; // auton score from human player
    frc::Pose2d TopLeftRed{};
    frc::Pose2d HumanPlayerIntakeAuto{};

    // RED POINTS
    frc::Pose2d left6{};
    frc::Pose2d right6{};
    frc::Pose2d left7{};
    frc::Pose2d right7{};
    frc::Pose2d left8{};
    frc::Pose2d right8{};
    frc::Pose2d left9{};
    frc::Pose2d right9{};
    frc::Pose2d left10{};
    frc::Pose2d right10{};
    frc::Pose2d left11{};
    frc::Pose2d right11{};
    
    frc::Pose2d left6L{};
    frc::Pose2d right6L{};
    frc::Pose2d left7L{};
    frc::Pose2d right7L{};
    frc::Pose2d left8L{};
    frc::Pose2d right8L{};
    frc::Pose2d left9L{};
    frc::Pose2d right9L{};
    frc::Pose2d left10L{};
    frc::Pose2d right10L{};
    frc::Pose2d left11L{};
    frc::Pose2d right11L{};

    // BLUE POINTS
    frc::Pose2d right17{};
    frc::Pose2d left17{};
    frc::Pose2d right18{}; 
    frc::Pose2d left18{};
    frc::Pose2d right19{};
    frc::Pose2d left19{};
    frc::Pose2d right20{};
    frc::Pose2d left20{};
    frc::Pose2d right21{};
    frc::Pose2d left21{};
    frc::Pose2d right22{};
    frc::Pose2d left22{};
    
    frc::Pose2d right17L{};
    frc::Pose2d left17L{};
    frc::Pose2d right18L{}; 
    frc::Pose2d left18L{};
    frc::Pose2d right19L{};
    frc::Pose2d left19L{};
    frc::Pose2d right20L{};
    frc::Pose2d left20L{};
    frc::Pose2d right21L{};
    frc::Pose2d left21L{};
    frc::Pose2d right22L{};
    frc::Pose2d left22L{};

     frc::Pose2d AutoAlignPose{}; 

    units::meter_t MOE{0.03}; //= 0.03_m; 
    units::degree_t MOErotation{1.5}; // = 1.5_deg; 
    units::time::second_t bufferTime{5.0};

    units::meter_t visionPoseOffsetX{0};
    units::meter_t visionPoseOffsetY{0};
    frc::Translation2d reefCenterBlue{4.4893_m, 4.0259_m};
    frc::Translation2d reefCenterRed = reefCenterBlue.RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg});

    bool isBlueAlliance = true; 
   
  
};