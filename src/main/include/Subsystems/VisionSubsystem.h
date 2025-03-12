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
    photon::PhotonCamera cameraOne{"camOne"};

  const frc::AprilTagFieldLayout kTagLayout{
    frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)};

frc::Transform3d robotToCam1 =
    frc::Transform3d(frc::Translation3d(0.16_m, 0.212_m, 0.21_m),
                    frc::Rotation3d(0_rad, -0.349_rad, 1.7453_rad));

  photon::PhotonCamera cameraTwo{"camTwo"};

frc::Transform3d robotToCam2 =
    frc::Transform3d(frc::Translation3d(0.16_m, 0.212_m, 0.21_m),
                    frc::Rotation3d(0_rad, -0.349_rad, -1.7453_rad));


photon::PhotonPoseEstimator poseEstimatorOne{kTagLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam1};
photon::PhotonPoseEstimator poseEstimatorTwo{kTagLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam2};


frc::Pose3d prevEstimatedRobotPose = frc::Pose3d{frc::Translation3d(0_m, 0_m, 0_m), frc::Rotation3d(0_rad, 0_rad, 0_rad)};

std::vector<photon::EstimatedRobotPose> getEstimatedGlobalPose(frc::Pose3d prevEstimatedRobotPose); 

 std::optional<photon::EstimatedRobotPose> EstimatedPose();

std::optional<frc::Pose3d> GetVisionPose();

private: 
    units::second_t lastProcessedTimeOne = 0_s;
    units::second_t lastProcessedTimeTwo = 0_s;

}; 