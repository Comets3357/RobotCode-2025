# pragma once 


#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>
#include <photon/targeting/PhotonTrackedTarget.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <Photon/PhotonPoseEstimator.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/Field2d.h>

class VisionSubsystem : public frc2::SubsystemBase 
{
    public: 
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

std::vector<frc::Pose3d> getEstimatedGlobalPose(frc::Pose3d prevEstimatedRobotPose); 

std::optional<frc::Pose3d> GetVisionPose();/* {

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
*/
}; 