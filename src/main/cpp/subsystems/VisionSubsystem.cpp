#include "subsystems/VisionSubsystem.h"

std::vector<photon::EstimatedRobotPose> VisionSubsystem::getEstimatedGlobalPose(frc::Pose3d prevEstimatedRobotPose) {

  std::vector<photon::PhotonPipelineResult> unreadResultsOne = cameraOne.GetAllUnreadResults();
  std::vector<photon::PhotonPipelineResult> unreadResultsTwo = cameraTwo.GetAllUnreadResults();
  std::optional<photon::EstimatedRobotPose> result1;
  std::optional<photon::EstimatedRobotPose> result2;
  std::vector<photon::EstimatedRobotPose> poses;

  poseEstimatorOne.SetReferencePose(prevEstimatedRobotPose);
  poseEstimatorTwo.SetReferencePose(prevEstimatedRobotPose);
  units::second_t currentTime = frc::Timer::GetFPGATimestamp();

  if (!unreadResultsOne.empty()) {
    auto cameraResults1 = unreadResultsOne[0];
    units::second_t frameTime1{cameraResults1.GetTimestamp().value()};


    if (cameraResults1.GetTimestamp().value() != 0) {
            units::second_t frameTime1{cameraResults1.GetTimestamp().value()};
            
            if (frameTime1 > lastProcessedTimeOne && frameTime1 <= currentTime) {
                result1 = poseEstimatorOne.Update(cameraResults1);
                lastProcessedTimeOne = frameTime1;  // Update last processed time
                fmt::print("YAY PROCESSED A FRAME\n");
            } else {
                fmt::print("Skipping outdated or duplicate frame from Camera 1\n");
            }
        }

  }

  if (unreadResultsTwo.size() > 0) {

  auto cameraResults2 = unreadResultsTwo[0];
  units::second_t frameTime2{cameraResults2.GetTimestamp().value()};


    if (cameraResults2.GetTimestamp().value() != 0) {
            units::second_t frameTime1{cameraResults2.GetTimestamp().value()};
            
            if (frameTime2 > lastProcessedTimeTwo && frameTime2 <= currentTime) {
                result2 = poseEstimatorTwo.Update(cameraResults2);
                lastProcessedTimeTwo = frameTime2;  // Update last processed time
                fmt::print("YAY PROCESSED A FRAME\n");
            } else {
                fmt::print("Skipping outdated or duplicate frame from Camera 1\n");
            }
        }
    
  }

  if (result1.has_value()) {
    poses.push_back(result1.value());
  } 
  if (result2.has_value()) {
    poses.push_back(result2.value());
  } 
  
  return poses;
}

// std::optional<photon::EstimatedRobotPose> VisionSubsystem::EstimatedPose() {
//   std::vector<photon::PhotonPipelineResult> unreadResultsOne = cameraOne.GetAllUnreadResults();
//   std::optional<photon::EstimatedRobotPose> visionEst;

//     // Run each new pipeline result through our pose estimator
//     for (const auto& result : cameraOne.GetAllUnreadResults()) {
//       // cache result and update pose estimator
//       visionEst = poseEstimatorOne.Update(result);
//       }

//   return visionEst; 
// }


