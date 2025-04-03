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
   // frc::SmartDashboard::PutData("Field No Vision", &m_fieldNoVision);

   // frc::SmartDashboard::PutData("Mirror Field", &m_mirrorField); 

    

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
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed)
    {
        isBlueAlliance = false; 
    } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
    {
        isBlueAlliance = true;
    }
    PoseEstimation();
    PoseEstimationNoVisionTest();
   // frc::SmartDashboard::PutNumber("Vision Offset X", visionPoseOffsetX.value());
    //frc::SmartDashboard::PutNumber("Vision Offset Y", visionPoseOffsetY.value());

    // frc::SmartDashboard::PutNumber("Gyro Yaw", units::degree_t(m_gyro.GetYaw()).value());
    
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

    // start std calculation
    double chassisSpeedSquared = pow((double) GetRobotRelativeSpeeds().vx, 2) + pow((double) GetRobotRelativeSpeeds().vy, 2); 
    double chassisSpeeds = pow(chassisSpeedSquared, 0.5); 
    double percentSpeed = (chassisSpeeds / 4.8); 
    double StdDev = (percentSpeed * (percentSpeed > 0.5) ? 30 : 15) + 0.1;

    double angSpeMulti = GetRobotRelativeSpeeds().omega / 0.0610865_rad_per_s; 

    for (int i = 0; i < std::ceil(angSpeMulti); i++)
    {
        StdDev += 5; 
    }

    double distancePose = (double)(units::meter_t{GetDistance(isBlueAlliance ? reefCenterBlue : reefCenterRed)} - 2.5_ft); 

    if (percentSpeed < 0.1)
    {
        StdDev = 0.35; 
    }
    else if (percentSpeed < 0.2){
        StdDev = 0.2 * pow(10, distancePose);
    } else if (percentSpeed < 0.40)
    {
        StdDev = pow(15, distancePose);
    } else{
        StdDev = 1000; 
    }

    if (frc::DriverStation::IsDisabled())
    {
        StdDev = 0.2; 
    }
    
        

   // frc::SmartDashboard::PutNumber("STDDEV", StdDev); 
    //frc::SmartDashboard::PutNumber("distancePose", distancePose); 


    
    m_poseEstimator.SetVisionMeasurementStdDevs({StdDev, StdDev, 100});
    // end std dev calculation
                            
    estimatedPoseVector = m_visionSubsystem.getEstimatedGlobalPose(frc::Pose3d{frc::Translation3d(0_m, 0_m, 0_m), frc::Rotation3d(0_rad, 0_rad, 0_rad)});

    // UPDATES VISION MEASUREMENTS BASED ON WHICH CAMERAS ARE GETTING UPDATES // 
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

void DriveSubsystem::GoToPos(frc::Pose2d targetPos, double max_output)
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
    frc::PIDController positionPID(5.0,0,0);
    frc::PIDController rotationPID(3.0,0,0);

    double speedX = positionPID.Calculate(deltaX, 0);
    double speedY = positionPID.Calculate(deltaY, 0);
    double angVel = rotationPID.Calculate(shortestRotation, 0); 

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        speedX *= -1;
        speedY *= -1;
    }
    
    double commanded_speed = std::sqrt(speedX * speedX + speedY * speedY);
    if (commanded_speed > max_output)
    {
        speedX = speedX * max_output / commanded_speed;
        speedY = speedY * max_output / commanded_speed;
    }

    Drive(units::meters_per_second_t{(speedX)}, units::meters_per_second_t{(speedY)}, -units::degrees_per_second_t{angVel}, true);
}

double DriveSubsystem::GetDistance(frc::Pose2d target)
{
    return (double)GetPose().Translation().Distance(target.Translation()); 
}

double DriveSubsystem::GetDistance(frc::Translation2d target)
{
    return (double)GetPose().Translation().Distance(target); 
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

 frc::Pose2d DriveSubsystem::findNearestTarget(bool isLeftSide)
 {
    frc::Pose2d temp{}; 
    double shortestDistance = 100000000; 
    
    if (isLeftSide)
    {
        if (isBlueAlliance)
        {
            double tempDist; 
            for (frc::Pose2d i: leftBluePoses)
            {
                tempDist = GetDistance(i); 
                
                if (tempDist < shortestDistance)
                {
                    shortestDistance = tempDist;
                    temp = i; 
                }
            }
        } else {
             double tempDist; 
            for (frc::Pose2d i: leftRedPoses)
            {
                tempDist = GetDistance(i); 
                
                if (tempDist < shortestDistance)
                {
                    shortestDistance = tempDist;
                    temp = i; 
                }
            }
        }
    } else {
         if (isBlueAlliance)
        {
             double tempDist; 
            for (frc::Pose2d i: rightBluePoses)
            {
                tempDist = GetDistance(i); 
                
                if (tempDist < shortestDistance)
                {
                    shortestDistance = tempDist;
                    temp = i; 
                }
            }
        } else {
             double tempDist; 
            for (frc::Pose2d i: rightRedPoses)
            {
                tempDist = GetDistance(i); 
                
                if (tempDist < shortestDistance)
                {
                    shortestDistance = tempDist;
                    temp = i; 
                }
            }
        }
    }

    double angleDiff = std::abs((double)(GetGyroHeading().Degrees() - temp.Rotation().Degrees())); 

    if (angleDiff > 180) angleDiff = 360 - angleDiff; 

    if (angleDiff > 90)
    {
       // frc::SmartDashboard::SmartDashboard::PutBoolean("FLIP SIDE", true); 
    if (temp == right6) temp = right6L;
    else if (temp == left6) temp = left6L;
    else if (temp == right7) temp = right7L;
    else if (temp == left7) temp = left7L;
    else if (temp == right8) temp = right8L;
    else if (temp == left8) temp = left8L;
    else if (temp == right9) temp = right9L;
    else if (temp == left9) temp = left9L;
    else if (temp == right10) temp = right10L;
    else if (temp == left10) temp = left10L;
    else if (temp == right11) temp = right11L;
    else if (temp == left11) temp = left11L;
    else if (temp == right17) temp = right17L;
    else if (temp == left17) temp = left17L;
    else if (temp == right18) temp = right18L;
    else if (temp == left18) temp = left18L;
    else if (temp == right19) temp = right19L;
    else if (temp == left19) temp = left19L;
    else if (temp == right20) temp = right20L;
    else if (temp == left20) temp = left20L;
    else if (temp == right21) temp = right21L;
    else if (temp == left21) temp = left21L;
    else if (temp == right22) temp = right22L;
    else if (temp == left22) temp = left22L;
    }
   // NearestTarget.SetRobotPose(temp); // Update the field with temp pose
  //  frc::SmartDashboard::SmartDashboard::PutBoolean("FLIP SIDE", false); 
    //frc::SmartDashboard::PutData("Nearest Target", &NearestTarget);

    return temp; 
 }

bool DriveSubsystem::ArmGoToLeftSide()
{
    frc::Translation2d transPos = (GetPose().Translation() - (isBlueAlliance ? reefCenterBlue : reefCenterRed));  
    double radians = GetPose().Rotation().Radians().value(); // Get heading in radians
    bool scoreLeftSide =  (std::cos(radians) * transPos.Y().value() - std::sin(radians) * transPos.X().value()) < 0;
   // frc::SmartDashboard::SmartDashboard::PutBoolean("SCORE LEFT SIDE", scoreLeftSide); 

    return scoreLeftSide; 

    //  (x1 * y2) - (y1 * x2) // get the determinant of the two vectors and return if it is positive or negative (booelean)
}

void DriveSubsystem::SetPointPositions()
{
    
    TopLeftRed = frc::Pose2d{12.48_m, 2.6848_m, frc::Rotation2d{150_deg}}; 
    BottomLeftRed =  frc::Pose2d{13.554352_m, 2.7912_m, frc::Rotation2d{-150_deg}}; 

    HumanPlayerIntakeAuto = frc::Pose2d{1.773_m, 6.92_m, frc::Rotation2d{35_deg}};   // blue
    HumanPlayerIntakeRight = frc::Pose2d{1.626_m, 1.026_m, frc::Rotation2d{145_deg}};   // blue

    HumanPlayerIntakeAutoRed =  frc::Pose2d{1.773_m, 7.0_m, frc::Rotation2d{35_deg}}.RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg}); // red
    HumanPlayerIntakeRightRed = frc::Pose2d{1.626_m, 1.026_m, frc::Rotation2d{145_deg}}.RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg}); // red





    // set points on blue side
    right18 = frc::Pose2d{3.02_m, 3.86_m, frc::Rotation2d{90_deg}};     // set points on blue side right
    left20 = frc::Pose2d{5.354_m, 5.197_m, frc::Rotation2d{-30_deg}};   // set points on blue side left

    right18L = frc::Pose2d{3.04_m, 3.82_m, frc::Rotation2d{270_deg}};
    left20L = frc::Pose2d{5.3007_m, 5.18932_m, frc::Rotation2d{150_deg}};

    right17 = right18.RotateAround(reefCenterBlue, frc::Rotation2d{60_deg}); 
    right22 = right18.RotateAround(reefCenterBlue, frc::Rotation2d{120_deg});
    right21 = right18.RotateAround(reefCenterBlue, frc::Rotation2d{180_deg});  
    right20 = right18.RotateAround(reefCenterBlue, frc::Rotation2d{240_deg}); 
    right19 = right18.RotateAround(reefCenterBlue, frc::Rotation2d{300_deg}); 
    left19 = left20.RotateAround(reefCenterBlue, frc::Rotation2d{60_deg}); 
    left18 = left20.RotateAround(reefCenterBlue, frc::Rotation2d{120_deg}); 
    left17 = left20.RotateAround(reefCenterBlue, frc::Rotation2d{180_deg}); 
    left22 = left20.RotateAround(reefCenterBlue, frc::Rotation2d{240_deg}); 
    left21 = left20.RotateAround(reefCenterBlue, frc::Rotation2d{300_deg}); 

    right17L = right18L.RotateAround(reefCenterBlue, frc::Rotation2d{60_deg}); 
    right22L = right18L.RotateAround(reefCenterBlue, frc::Rotation2d{120_deg});
    right21L = right18L.RotateAround(reefCenterBlue, frc::Rotation2d{180_deg});  
    right20L = right18L.RotateAround(reefCenterBlue, frc::Rotation2d{240_deg}); 
    right19L = right18L.RotateAround(reefCenterBlue, frc::Rotation2d{300_deg}); 
    left19L = left20L.RotateAround(reefCenterBlue, frc::Rotation2d{60_deg}); 
    left18L = left20L.RotateAround(reefCenterBlue, frc::Rotation2d{120_deg}); 
    left17L = left20L.RotateAround(reefCenterBlue, frc::Rotation2d{180_deg}); 
    left22L = left20L.RotateAround(reefCenterBlue, frc::Rotation2d{240_deg}); 
    left21L = left20L.RotateAround(reefCenterBlue, frc::Rotation2d{300_deg}); 

        // set points on red side
    left7 =  left18.RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg}); 
    right7 = right18.RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg});

    left7L = left18L.RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg});
    right7L = right18L.RotateAround(frc::Translation2d{8.774176_m, 4.0259_m}, frc::Rotation2d{180_deg});

    left8 = left7.RotateAround(reefCenterRed, frc::Rotation2d{60_deg});
    right8 = right7.RotateAround(reefCenterRed, frc::Rotation2d{60_deg});
    left9 = left7.RotateAround(reefCenterRed, frc::Rotation2d{120_deg});
    right9 = right7.RotateAround(reefCenterRed, frc::Rotation2d{120_deg});
    left10 = left7.RotateAround(reefCenterRed, frc::Rotation2d{180_deg});
    right10 = right7.RotateAround(reefCenterRed, frc::Rotation2d{180_deg});
    left11 = left7.RotateAround(reefCenterRed, frc::Rotation2d{240_deg});
    right11 = right7.RotateAround(reefCenterRed, frc::Rotation2d{240_deg});
    left6 = left7.RotateAround(reefCenterRed, frc::Rotation2d{300_deg});
    right6 = right7.RotateAround(reefCenterRed, frc::Rotation2d{300_deg});

    left8L = left7L.RotateAround(reefCenterRed, frc::Rotation2d{60_deg});
    right8L = right7L.RotateAround(reefCenterRed, frc::Rotation2d{60_deg});
    left9L = left7L.RotateAround(reefCenterRed, frc::Rotation2d{120_deg});
    right9L = right7L.RotateAround(reefCenterRed, frc::Rotation2d{120_deg});
    left10L = left7L.RotateAround(reefCenterRed, frc::Rotation2d{180_deg});
    right10L = right7L.RotateAround(reefCenterRed, frc::Rotation2d{180_deg});
    left11L = left7L.RotateAround(reefCenterRed, frc::Rotation2d{240_deg});
    right11L = right7L.RotateAround(reefCenterRed, frc::Rotation2d{240_deg});
    left6L = left7L.RotateAround(reefCenterRed, frc::Rotation2d{300_deg});
    right6L = right7L.RotateAround(reefCenterRed, frc::Rotation2d{300_deg});

    
    // creates vector for closest tag function
    rightRedPoses.push_back(right6);
    rightRedPoses.push_back(right7);
    rightRedPoses.push_back(right8);
    rightRedPoses.push_back(right9);
    rightRedPoses.push_back(right10);
    rightRedPoses.push_back(right11);

    leftRedPoses.push_back(left6);
    leftRedPoses.push_back(left7);
    leftRedPoses.push_back(left8);
    leftRedPoses.push_back(left9);
    leftRedPoses.push_back(left10);
    leftRedPoses.push_back(left11);
    
    rightBluePoses.push_back(right17); 
    rightBluePoses.push_back(right18); 
    rightBluePoses.push_back(right19); 
    rightBluePoses.push_back(right20); 
    rightBluePoses.push_back(right21); 
    rightBluePoses.push_back(right22); 

    leftBluePoses.push_back(left17); 
    leftBluePoses.push_back(left18); 
    leftBluePoses.push_back(left19); 
    leftBluePoses.push_back(left20); 
    leftBluePoses.push_back(left21); 
    leftBluePoses.push_back(left22); 


}
