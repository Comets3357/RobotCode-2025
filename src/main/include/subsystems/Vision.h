// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/DriveSubsystem.h"


#include "RobotContainer.h"
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>
#include <photon/targeting/PhotonTrackedTarget.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <Photon/PhotonPoseEstimator.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/estimator/PoseEstimator.h>


class Vision : public frc2::SubsystemBase
{
public:
std::vector<frc::Pose3d> getEstimatedGlobalPose(frc::Pose3d prevEstimatedRobotPose) {

  std::vector<photon::PhotonPipelineResult> unreadResultsOne = cameraOne.GetAllUnreadResults();
  std::vector<photon::PhotonPipelineResult> unreadResultsTwo = cameraTwo.GetAllUnreadResults();
  units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
  std::optional<photon::EstimatedRobotPose> result1;
  std::optional<photon::EstimatedRobotPose> result2;
  std::vector<frc::Pose3d> poses;

  poseEstimatorOne.SetReferencePose(prevEstimatedRobotPose);
  poseEstimatorTwo.SetReferencePose(prevEstimatedRobotPose);


  if (unreadResultsOne.size() > 0) {
    cameraResults = unreadResultsOne.at(0);
    result1 = poseEstimatorOne.Update(cameraResults);
  }

  if (unreadResultsTwo.size() > 0) {
    cameraResults = unreadResultsTwo.at(0);
    result2 = poseEstimatorTwo.Update(cameraResults);
    
  }

  if (result1.has_value()) {
    poses.push_back(result1.value().estimatedPose);
  } else if (result2.has_value()) {
    poses.push_back(result2.value().estimatedPose);
  } else {

  }
  return poses;
}


private:
    std::optional<frc2::CommandPtr> m_autonomousCommand;


    RobotContainer m_container;
    frc::Field2d m_field;

  photon::PhotonCamera cameraOne{"photonvision"};

  const frc::AprilTagFieldLayout kTagLayout{
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)};

frc::Transform3d robotToCam1 =
    frc::Transform3d(frc::Translation3d(0_m, 0_m, 0_m),
                    frc::Rotation3d(0_rad, 0_rad, 0_rad));

  photon::PhotonCamera cameraTwo{"testCamera"};

frc::Transform3d robotToCam2 =
    frc::Transform3d(frc::Translation3d(0_m, 0_m, 0_m),
                    frc::Rotation3d(0.5_rad, 0_rad, 0_rad));

            

photon::PhotonPoseEstimator poseEstimatorOne{kTagLayout, photon::PoseStrategy::CLOSEST_TO_LAST_POSE, robotToCam1};

photon::PhotonPoseEstimator poseEstimatorTwo{kTagLayout, photon::PoseStrategy::CLOSEST_TO_LAST_POSE, robotToCam2};





photon::PhotonPipelineResult cameraResults;


frc::Pose3d prevEstimatedRobotPose = frc::Pose3d{frc::Translation3d(0_m, 0_m, 0_m), frc::Rotation3d(0_rad, 0_rad, 0_rad)};


// to do: use AddVisionMeasurement() to add vision to robot swerve pose estimator
// to do: autoAlign to position on the field using robot pose and vision. Give robot a position and move or align to it
// to do: use the robot pose to move the robot to a position on the field

};
