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
    frc::SmartDashboard::PutData("Field No Vision", &m_fieldNoVision);

    frc::SmartDashboard::PutData("Mirror Field", &m_mirrorField); 

    

    // Configure the AutoBuilder last
    RobotConfig config = RobotConfig::fromGUISettings();

        AutoBuilder::configure(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ if (frc::DriverStation::GetAlliance().has_value()) {frc::Pose2d temp{pose.X(), pose.Y(), (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) ? GetGyroHeading().RotateBy(frc::Rotation2d{180_deg}) : GetGyroHeading()}; m_poseEstimator.ResetPose(temp);} }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return GetRobotRelativeSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](auto speeds, auto feedforwards){ DriveFromChassisSpeeds(speeds, false); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        std::make_shared<PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PIDConstants(22.5, 0.0, 0.0), // Translation PID constants
            PIDConstants(10, 0.0, 0.0) // Rotation PID constants
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
     if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed && initVisionUse < 1)
    {
        reefCenterBlue = reefCenterBlue.RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg}); 
        initVisionUse += 1; 
    }
    PoseEstimation();
    PoseEstimationNoVisionTest();
    frc::SmartDashboard::PutNumber("Vision Offset X", visionPoseOffsetX.value());
    frc::SmartDashboard::PutNumber("Vision Offset Y", visionPoseOffsetY.value());
    // frc::SmartDashboard::PutNumber("Bottom red left X", (double)BottomLeftRed.X());
    // frc::SmartDashboard::PutNumber("Bottom red left Y", (double)BottomLeftRed.Y());

    // frc::SmartDashboard::PutNumber("Top red left X", (double)TopLeftRed.X());
    // frc::SmartDashboard::PutNumber("Top red left Y", (double)TopLeftRed.Y());
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

    double chassisSpeedSquared = pow((double) GetRobotRelativeSpeeds().vx, 2) + pow((double) GetRobotRelativeSpeeds().vy, 2); 
    double chassisSpeeds = pow(chassisSpeedSquared, 0.5); 
    double percentSpeed = (chassisSpeeds / 4.8); 
    double StdDev = (percentSpeed * (percentSpeed > 0.5) ? 30 : 15) + 0.1;

    double angSpeMulti = GetRobotRelativeSpeeds().omega / 0.0610865_rad_per_s; 

    for (int i = 0; i < std::ceil(angSpeMulti); i++)
    {
        StdDev += 5; 
    }

    double distancePose = (double)(GetPose().Translation().Distance(reefCenterBlue) - 2.5_ft/*BLUE REEF*/ ); 
    

    if (distancePose > 3)
    {
        StdDev += 500; 
    } 
    else if (distancePose > 2)
    {
        StdDev += 50;
    }

    if (distancePose < 1 && chassisSpeeds < 0.5)
    {
        StdDev = 0.9;
    }

    if (frc::DriverStation::IsDisabled())
    {
        StdDev = 1.2; 
    }
        

    frc::SmartDashboard::PutNumber("STDDEV", StdDev); 
    frc::SmartDashboard::PutNumber("distancePose", distancePose); 


    
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

void DriveSubsystem::PoseEstimationNoVisionTest()
{
    auto alliance = frc::DriverStation::GetAlliance();
    if (alliance == frc::DriverStation::Alliance::kRed)
    {
        m_poseEstimatorNoVision.Update(m_gyro.Get2DRotation().RotateBy(frc::Rotation2d(180_deg)),
                           {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                            m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
    }
    else
    {
        m_poseEstimatorNoVision.Update(m_gyro.Get2DRotation(),
                           {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                            m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
    }

    m_fieldNoVision.SetRobotPose(m_poseEstimatorNoVision.GetEstimatedPosition());

}

void DriveSubsystem::UpdateNonVisionPose() {
    m_poseEstimatorNoVision.ResetPose(m_poseEstimator.GetEstimatedPosition());
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
    frc::PIDController positionPID(1.25,0,0);
    frc::PIDController rotationPID(1,0,0);

    double speedX = positionPID.Calculate(deltaX, 0);
    double speedY = positionPID.Calculate(deltaY, 0);
    double angVel = rotationPID.Calculate(shortestRotation, 0); 

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        speedX *= -1;
        speedY *= -1;
    }
    
    if (std::abs(speedX) < 0.02 && std::abs(speedY) < 0.02) {
       speedX = 0;
       speedY = 0;  
    }
    if (std::abs(angVel) < 0.005) {
        angVel = 0;
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
    TopLeftBlue = frc::Pose2d{5.334_m, 5.197_m, frc::Rotation2d{-30_deg}}; 
    BottomLeftBlue = frc::Pose2d{3.61_m, 5.08_m, frc::Rotation2d{30_deg}}; 
    //BottomLeftBlue.RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg});//frc::Pose2d{4.982_m, 5.392_m, frc::Rotation2d{30_deg}}; // these are not right
    
    TopLeftRed = frc::Pose2d{12.48_m, 2.6848_m, frc::Rotation2d{150_deg}}; //TopLeftBlue.RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg});//frc::Pose2d{5.334_m, 5.197_m, frc::Rotation2d{-30_deg}}; // this is not right 
    BottomLeftRed =  frc::Pose2d{13.554352_m, 2.7912_m, frc::Rotation2d{-150_deg}}; 
    //TestingPointRed = frc::Pose2d{15_m, 4.130_m, frc::Rotation2d{90_deg}};
    TestingPointRed = frc::Pose2d{15.55_m, -5.45_m, -138_deg}; 

    left9 = frc::Pose2d{12.33_m, 5.14_m, frc::Rotation2d{30_deg}};//TopLeftRed.RotateAround(reefCenterRed, frc::Rotation2d{-120_deg}); 
    right8 = frc::Pose2d{13.63_m, 5.34_m, frc::Rotation2d{-30_deg}};
}
