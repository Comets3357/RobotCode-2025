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
    cameraResults = unreadResultsOne.at(0);
    result1 = poseEstimatorOne.Update(cameraResults);
  }

  if (unreadResultsTwo.size() > 0) {
    cameraResults = unreadResultsTwo.at(0);
    result2 = poseEstimatorTwo.Update(cameraResults);
    
  }

  if (result1.has_value()) {
    poses.push_back(result1.value().estimatedPose);
  } 
  if (result2.has_value()) {
    poses.push_back(result2.value().estimatedPose);
  } 
  
  return poses;
}

std::optional<frc::Pose3d> VisionSubsystem::GetVisionPose() {
    // Get vision results from both cameras
    std::vector<photon::PhotonPipelineResult> resultsOne = cameraOne.GetAllUnreadResults();
    std::vector<photon::PhotonPipelineResult> resultsTwo = cameraTwo.GetAllUnreadResults();

    std::optional<photon::EstimatedRobotPose> result1;
    std::optional<photon::EstimatedRobotPose> result2;

    // Pose storage
    std::vector<frc::Pose3d> poses;

    // Process results from Camera One
    if (!resultsOne.empty()) {
        cameraResults = resultsOne.at(0);  // Take the first result
        result1 = poseEstimatorOne.Update(cameraResults);
    }

    // Process results from Camera Two
    if (!resultsTwo.empty()) {
        cameraResults = resultsTwo.at(0);  // Take the first result
        result2 = poseEstimatorTwo.Update(cameraResults);
    }

    // Add results to the pose vector
    if (result1.has_value()) {
        poses.push_back(result1.value().estimatedPose);
    }

    if (result2.has_value()) {
        poses.push_back(result2.value().estimatedPose);
    }

    // Combine poses into a single one, or return the best one (you could apply weights or filters here)
    if (!poses.empty()) {
        // In this case, just return the first available pose
        // You could apply fusion logic here to select the best estimate
        return poses.at(0);
    }

    // No valid vision pose was found
    return std::nullopt;
}