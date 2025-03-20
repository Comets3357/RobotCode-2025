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
                  kRearRightChassisAngularOffset}
{
    SetPointPositions(); 
    // Usage reporting for MAXSwerve template
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
               HALUsageReporting::kRobotDriveSwerve_MaxSwerve);

    redux::canand::EnsureCANLinkServer();

    frc::SmartDashboard::PutData("Field", &m_field);
    frc::SmartDashboard::PutData("Mirror Field", &m_mirrorField); 

    

    // Configure the AutoBuilder last
    RobotConfig config = RobotConfig::fromGUISettings();

        AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ if (frc::DriverStation::GetAlliance().has_value()) {frc::Pose2d temp{pose.X(), pose.Y(), (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) ? GetGyroHeading().RotateBy(frc::Rotation2d{180_deg}) : GetGyroHeading()}; m_poseEstimator.ResetPose(temp);} }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ DriveFromChassisSpeeds(speeds, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(7.5, 0.0, 0.0), // Translation PID constants
            PIDConstants(4.0, 0.0, 0.0) // Rotation PID constants
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
    return m_gyro.Get2DRotation(); 
}

double DriveSubsystem::GetChassisSpeed()
{
    return (double)m_frontLeft.GetState().speed; 
}

void DriveSubsystem::Periodic()
{
    // Implementation of subsystem periodic method goes here.
    PoseEstimation();
    frc::SmartDashboard::PutNumber("Vision Offset X", visionPoseOffsetX.value());
    frc::SmartDashboard::PutNumber("Vision Offset Y", visionPoseOffsetY.value());
    // frc::SmartDashboard::PutNumber("Gyro Yaw", units::degree_t(m_gyro.GetYaw()).value());
    // frc::SmartDashboard::PutNumber("Drive X (m):", m_poseEstimator.GetPose().Translation().X().value());
    
}

void DriveSubsystem::PoseEstimation() {
   
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance == frc::DriverStation::Alliance::kRed)
    {
        m_poseEstimator.Update(m_gyro.Get2DRotation().RotateBy(frc::Rotation2d(180_deg)),
                           {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                            m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
    }
    else
    {
        m_poseEstimator.Update(m_gyro.Get2DRotation(),
                           {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                            m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
    }

    double chassisSpeedSquared= pow((double) GetRobotRelativeSpeeds().vx, 2) + pow((double) GetRobotRelativeSpeeds().vy, 2); 
    double chassisSpeeds = pow(chassisSpeedSquared, 0.5); 
    double StdDev = ((chassisSpeeds / 4.8) * 3.0) + 0.2;
    
    
    m_poseEstimator.SetVisionMeasurementStdDevs({StdDev, StdDev, 100});
                            
    estimatedPoseVector = m_visionSubsystem.getEstimatedGlobalPose(frc::Pose3d{frc::Translation3d(0_m, 0_m, 0_m), frc::Rotation3d(0_rad, 0_rad, 0_rad)});

    if (estimatedPoseVector.size() == 0)
    {

    } 
    if (estimatedPoseVector.size() == 1)
    {
        EstPose1 = estimatedPoseVector.at(0).estimatedPose.ToPose2d();

        if (alliance == frc::DriverStation::Alliance::kRed) {
        EstPose1 = frc::Pose2d{EstPose1.X(), EstPose1.Y(), m_gyro.Get2DRotation().RotateBy(frc::Rotation2d(180_deg))};
        }
        else {
            EstPose1 = frc::Pose2d{EstPose1.X(), EstPose1.Y(), m_gyro.Get2DRotation()};
        }
        
        if (EstPose1.X() > 0.39743_m && EstPose1.X() < 17.15_m && EstPose1.Y() > 0.39743_m && EstPose1.Y() < 7.655_m)
        {
            m_poseEstimator.AddVisionMeasurement(EstPose1, estimatedPoseVector.at(0).timestamp); 
        }
    } else if (estimatedPoseVector.size() == 2)
    {
        EstPose1 = estimatedPoseVector.at(0).estimatedPose.ToPose2d();
        EstPose2 = estimatedPoseVector.at(1).estimatedPose.ToPose2d();
        
        if (alliance == frc::DriverStation::Alliance::kRed) {
        EstPose1 = frc::Pose2d{EstPose1.X(), EstPose1.Y(), m_gyro.Get2DRotation().RotateBy(frc::Rotation2d(180_deg))};
        EstPose2 = frc::Pose2d{EstPose2.X(), EstPose2.Y(), m_gyro.Get2DRotation().RotateBy(frc::Rotation2d(180_deg))};
        }
        else {
            EstPose1 = frc::Pose2d{EstPose1.X(), EstPose1.Y(), m_gyro.Get2DRotation()};
            EstPose2 = frc::Pose2d{EstPose2.X(), EstPose2.Y(), m_gyro.Get2DRotation()};
        }

        if (EstPose1.X() > 0.39743_m && EstPose1.X() < 17.15_m && EstPose1.Y() > 0.39743_m && EstPose1.Y() < 7.655_m)
        {
            m_poseEstimator.AddVisionMeasurement(EstPose1, estimatedPoseVector.at(0).timestamp);
        }
        if (EstPose2.X() > 0.39743_m && EstPose2.X() < 17.15_m && EstPose2.Y() > 0.39743_m && EstPose2.Y() < 7.655_m)
        {
            m_poseEstimator.AddVisionMeasurement(EstPose2, estimatedPoseVector.at(1).timestamp); 
        }
    }

    m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());
    m_mirrorField.SetRobotPose(m_poseEstimator.GetEstimatedPosition().RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg})); 
    frc::SmartDashboard::PutNumber("X (in)", ((units::inch_t)m_poseEstimator.GetEstimatedPosition().X()).value());
    frc::SmartDashboard::PutNumber("Y (in)", ((units::inch_t)m_poseEstimator.GetEstimatedPosition().Y()).value());
    frc::SmartDashboard::PutNumber("Rot (degrees)", ((units::degree_t)m_poseEstimator.GetEstimatedPosition().Rotation().Degrees()).value());
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

void DriveSubsystem::ZeroHeading() 
{ 
    m_gyro.ZeroGyro(); 
    
    frc::Pose2d newPose;
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance == frc::DriverStation::Alliance::kRed)
    {
        newPose = frc::Pose2d{m_poseEstimator.GetEstimatedPosition().Translation(), frc::Rotation2d{180_deg}};
    } else {
        newPose = frc::Pose2d{m_poseEstimator.GetEstimatedPosition().Translation(), frc::Rotation2d{0_deg}};

    }
    m_poseEstimator.ResetPose(newPose);
    gyroZero = true;
}

double DriveSubsystem::GetTurnRate()
{
    return m_gyro.GetAngularVelocityYaw().value(); //-m_gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_poseEstimator.GetEstimatedPosition(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
    units::degree_t angle{pose.Rotation().Degrees()}; 
    m_gyro.SetAngle(angle); 
    m_poseEstimator.ResetPosition(
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

void DriveSubsystem::GoToPos(frc::Pose2d targetPos)
{

    
    frc::Pose2d currentPos = GetPose();

    frc::Pose2d newTargetPos{targetPos.X() + visionPoseOffsetX, targetPos.Y() + visionPoseOffsetY, targetPos.Rotation()};


    
    // frc::Translation2d translate{targetPos.X()-currentPos.X(), targetPos.Y()-currentPos.Y()}

    double deltaX = (double)(newTargetPos.X() -currentPos.X());
    double deltaY = (double)(newTargetPos.Y() + visionPoseOffsetY-currentPos.Y());

    double current = (double)currentPos.Rotation().Degrees(); 
    double target = (double)targetPos.Rotation().Degrees(); 

    double shortestRotation = 0; 
    double delta = std::fmod((target-current) + 180, 360) - 180;
    shortestRotation = (delta < -180) ? delta + 360 : delta;

    // double x = (double)currentPos.distance(newTargetPos).value();
    // double p = 2; 

    // if ( x < 0.5)
    // {
    //     p = 1; 
    // } 
    frc::PIDController positionPID(2,0,0);

      double speedX = positionPID.Calculate(deltaX, 0);
    double speedY = positionPID.Calculate(deltaY, 0);
    double angVel = positionPID.Calculate(shortestRotation, 0); 

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        speedX *= -1;
        speedY *= -1;
    }
    

   

  

    Drive(units::meters_per_second_t{(speedX)}, units::meters_per_second_t{(speedY)}, -units::degrees_per_second_t{angVel}, true);
}

bool DriveSubsystem::inRange(frc::Pose2d driverPose, frc::Pose2d pose1, units::meter_t MOE, units::angle::degree_t MOEangle)
{
        bool xInRange = false;
        bool yInRange = false;
        bool angleInRange = false; 

        if ((double)driverPose.X() > (double)(pose1.X() - MOE) && (double)driverPose.X() < (double)(pose1.X() + MOE))
        {
            xInRange = true;
        }
        else 
        {
            xInRange = false; 
        }

        if ((double)driverPose.Y() > (double)(pose1.Y() - MOE) && (double)driverPose.Y() < (double)(pose1.Y() + MOE))
        {
            yInRange = true;
        }
        else 
        {
            yInRange = false; 
        }

        if ((double)driverPose.Rotation().Degrees() > (double)pose1.Rotation().Degrees() - (double)MOEangle && (double)driverPose.Rotation().Degrees() < (double)pose1.Rotation().Degrees() + (double)MOEangle)
        {
            angleInRange = true; 
        }
        else
        {
            angleInRange = false; 
        }

        return xInRange && yInRange && angleInRange; 
}

void DriveSubsystem::SetPointPositions()
{
    BottomLeftRed = frc::Pose2d{4.982_m, 5.392_m, frc::Rotation2d{30_deg}}; // these are not right
    TopLeftRed = frc::Pose2d{5.334_m, 5.197_m, frc::Rotation2d{-30_deg}}; // this is not right 
    TopLeftBlue = frc::Pose2d{5.334_m, 5.197_m, frc::Rotation2d{-30_deg}}; 
    BottomLeftBlue = frc::Pose2d{4.982_m, 5.392_m, frc::Rotation2d{30_deg}}; 

}
