#include "subsystems/VisionSubsystem.h"

std::vector<frc::Pose3d> VisionSubsystem::getEstimatedGlobalPose(frc::Pose3d prevEstimatedRobotPose) {

  std::vector<photon::PhotonPipelineResult> unreadResultsOne = cameraOne.GetAllUnreadResults();
  std::vector<photon::PhotonPipelineResult> unreadResultsTwo = cameraTwo.GetAllUnreadResults();
  units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
  std::optional<photon::EstimatedRobotPose> result1;
  std::optional<photon::EstimatedRobotPose> result2;
  std::vector<frc::Pose3d> poses;

  poseEstimatorOne.SetReferencePose(prevEstimatedRobotPose);
  poseEstimatorTwo.SetReferencePose(prevEstimatedRobotPose);


  if (unreadResultsOne.size() > 0) {
  auto cameraResults1 = unreadResultsOne.at(0);
  result1 = poseEstimatorOne.Update(cameraResults1);
  }

  if (unreadResultsTwo.size() > 0) {
  auto cameraResults2 = unreadResultsTwo.at(0);
  result2 = poseEstimatorTwo.Update(cameraResults2);
    
  }

  if (result1.has_value()) {
    poses.push_back(result1.value().estimatedPose);
  } 
  if (result2.has_value()) {
    poses.push_back(result2.value().estimatedPose);
  } 
  
  return poses;
}

std::optional<photon::EstimatedRobotPose> VisionSubsystem::EstimatedPose() {
  std::vector<photon::PhotonPipelineResult> unreadResultsOne = cameraOne.GetAllUnreadResults();
  std::optional<photon::EstimatedRobotPose> visionEst;

    // Run each new pipeline result through our pose estimator
    for (const auto& result : cameraOne.GetAllUnreadResults()) {
      // cache result and update pose estimator
      visionEst = poseEstimatorOne.Update(result);
      }

  return visionEst; 
}


