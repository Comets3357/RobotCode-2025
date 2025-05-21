#include "Commands/DriverCommands.h"

                                                                    
void DriverCommands(DriveSubsystem* m_drive, ClimbSubsystem* m_climb, ElevatorSubsystem* m_elevator,
                    ElbowSubsystem* m_elbow, IntakeSubsystem* m_intake, LEDSubsystem* m_LED, 
                    frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController) {

    //  _____       _                  ____        _   _                  
    // |  __ \     (_)                |  _ \      | | | |                 
    // | |  | |_ __ ___   _____ _ __  | |_) |_   _| |_| |_ ___  _ __  ___ 
    // | |  | | '__| \ \ / / _ \ '__| |  _ <| | | | __| __/ _ \| '_ \/ __|
    // | |__| | |  | |\ V /  __/ |    | |_) | |_| | |_| || (_) | | | \__ |
    // |_____/|_|  |_| \_/ \___|_|    |____/ \__,_|\__|\__\___/|_| |_|___/
                                                                    

    //Zeroes the gyro for driving convenience
    m_driverController->Start().OnTrue(frc2::cmd::RunOnce([=] {m_drive->ZeroHeading();}).IgnoringDisable(true)); 

    //Functions to drive the swerve modules, adds a conditional for speed reduction.
    m_drive->SetDefaultCommand(frc2::RunCommand(
    [=] {

        if (!(m_drive->halfSpeed)) {
        m_drive->Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController->GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController->GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController->GetRightX(), OIConstants::kDriveDeadband)},
            true);
        }
        else 
        {
            //divides the values from the joysticks by 3 to reduce the speed.
            m_drive->Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController->GetLeftY(), OIConstants::kDriveDeadband) / 3.0 },
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController->GetLeftX(), OIConstants::kDriveDeadband) / 3.0 },
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController->GetRightX(), OIConstants::kDriveDeadband) / 3.0 },
            true);
        }
    }, {m_drive})
    );

    //RIGHT TRIGGER
    //Halves the speed of swerve 
    m_driverController->RightTrigger().OnTrue(frc2::cmd::RunOnce([=] {m_drive->halfSpeed = true;})); 
    m_driverController->RightTrigger().OnFalse(frc2::cmd::RunOnce([=] {m_drive->halfSpeed = false;}));

//   m_driverController->A().WhileTrue(frc2::cmd::Run([=] {m_drive->GoToPos(m_drive->left17);}, {m_drive})); 
//   m_driverController->B().WhileTrue(frc2::cmd::Run([=] {m_drive->GoToPos(m_drive->left19);}, {m_drive})); 
//   m_driverController->X().WhileTrue(frc2::cmd::Run([=] {m_drive->GoToPos(m_drive->left18);}, {m_drive})); 
//   m_driverController->Y().WhileTrue(frc2::cmd::Run([=] {m_drive->GoToPos(m_drive->left22);}, {m_drive})); 
  
   m_driverController->RightBumper().WhileTrue(frc2::cmd::Run([=] {m_drive->AutoAlignPose = m_drive->findNearestTarget(false); m_drive->GoToPos(m_drive->AutoAlignPose); m_drive->isAutoAligning = true; }, {m_drive})); 
   m_driverController->RightBumper().OnFalse(frc2::cmd::RunOnce([=]{m_drive->AutoAlignPose = frc::Pose2d{}; m_drive->isAutoAligning = false;})); 
   m_driverController->LeftBumper().WhileTrue(frc2::cmd::Run([=] {m_drive->AutoAlignPose = m_drive->findNearestTarget(true); m_drive->GoToPos(m_drive->AutoAlignPose); m_drive->isAutoAligning = true;}, {m_drive})); 
   m_driverController->LeftBumper().OnFalse(frc2::cmd::RunOnce([=]{m_drive->AutoAlignPose = frc::Pose2d{}; m_drive->isAutoAligning = false;})); 



   //m_driverController->B().OnTrue(frc2::cmd::RunOnce([=] {m_drive->UpdateNonVisionPose();}, {m_drive}));
    //m_driverController->A().OnFalse(frc2::cmd::RunOnce([=] {m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, true);}, {m_drive})); 

   // m_driverController->B().WhileTrue(frc2::cmd::Run([=] {m_drive->GoToPos((frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) ? m_drive->TopLeftBlue : m_drive->TopLeftRed);}, {m_drive})); 
    //m_driverController->B().OnFalse(frc2::cmd::RunOnce([=] {m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, true);}, {m_drive})); 

    // m_driverController->POVUp().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetY -= 0.01_m * 0.5; m_drive->visionPoseOffsetX += 0.01_m * 0.866;}));
    // m_driverController->POVDown().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetY += 0.01_m * 0.5; m_drive->visionPoseOffsetX -= 0.01_m * 0.866;}));
    // m_driverController->POVLeft().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetX += 0.01_m * 0.5; m_drive->visionPoseOffsetY += 0.01_m * 0.866;}));
    // m_driverController->POVRight().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetX -= 0.01_m * 0.5; m_drive->visionPoseOffsetY -= 0.01_m * 0.866;}));

    //  m_driverController->POVUp().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetX += 0.01_m;}));
    // m_driverController->POVDown().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetX -= 0.01_m;}));
    // m_driverController->POVLeft().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetY += 0.01_m;}));
    // m_driverController->POVRight().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetY -= 0.01_m;}));
    //   ____  _   _                 ____        _   _             
    //  / __ \| | | |               |  _ \      | | | |                 
    // | |  | | |_| |__   ___ _ __  | |_) |_   _| |_| |_ ___  _ __  ___ 
    // | |  | | __| '_ \ / _ \ '__| |  _ <| | | | __| __/ _ \| '_ \/ __|
    // | |__| | |_| | | |  __/ |    | |_) | |_| | |_| || (_) | | | \__ |
    //  \____/ \__|_| |_|\___|_|    |____/ \__,_|\__|\__\___/|_| |_|___/

    
    // LEFT TRIGGER // human player signal it will flash purple 
   // m_driverController->LeftTrigger().OnTrue(frc2::cmd::RunOnce([=] {m_LED->hPlayer = true;}));
    //m_driverController->LeftTrigger().OnFalse(frc2::cmd::RunOnce([=] {m_LED->hPlayer = false;}));

    //LEFT BUMPER  // human player signal to ground it will flash yellow // 

    // m_driverController->LeftBumper().OnTrue(frc2::cmd::RunOnce([=] {m_LED->hPlayerGround = true;}));
    // m_driverController->LeftBumper().OnFalse(frc2::cmd::RunOnce([=] {m_LED->hPlayerGround = false;}));



    //RIGHT BUMPER
    //Auto Aligns robot to a certain angle
    //preferably used to align robot to human player station
   // m_driverController->RightBumper().WhileTrue(rotateTo(m_drive, 144_deg, m_driverController));

    //LEFT BUMPER
    //Auto Aligns robot to a certain angle
    //preferably used to align robot to human player station
    //m_driverController->LeftBumper().WhileTrue(rotateTo(m_drive, 36_deg, m_driverController));

}

frc2::CommandPtr rotateTo(DriveSubsystem *drive, units::degree_t targetrot, frc2::CommandXboxController *m_driverController) {
    return frc2::cmd::Run([drive, targetrot, m_driverController] {
        drive->Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController->GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController->GetLeftX(), OIConstants::kDriveDeadband)},
            units::degrees_per_second_t{shortestRotation(drive->GetGyroHeading().Degrees().value(), targetrot.value())}*1.2,
        true);}, {drive});
}

double shortestRotation(double current, double target) {
    double delta = std::fmod((target-current) + 180, 360) - 180;
    return (delta < -180) ? delta + 360 : delta;
}

frc2::CommandPtr defaultBenchTest(DriveSubsystem* m_drive, ClimbSubsystem* m_climb, ElevatorSubsystem* m_elevator,
                    ElbowSubsystem* m_elbowSubsystem, IntakeSubsystem* m_intake,
                    frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController)
{
    return frc2::cmd::Run([=] {
        m_drive->Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController->GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController->GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController->GetRightX(), OIConstants::kDriveDeadband)},
            true);
    }, {m_drive}).RaceWith(frc2::cmd::Wait(2_s))
    .AndThen(frc::cmd::Run([=] {m_elevator->setSpeed(0.2), {m_elevator}}))
    .AlongWith(frc2::cmd::WaitUntil([=]{ return m_elevator->getAPosition()>49.5;}))
    .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); },{m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (3.5);}))
    .AndThen(frc2::cmd::RunOnce([=] { m_elevator->setPosition(50);}, {m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil([=]{ return m_elevator->getAPosition()>49.5;}))
    .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); },{m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (3.5);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(295); m_elbow->setWristAngle(0); m_elbow->setRollerSpeed(0.4); m_elevator->setPosition(3);}, {m_elbow, m_elevator})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return (m_elbow->getWristAngle() < 2) && (m_elevator->getAPosition() < 5);}))
    .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(305);}, {m_elbow}))
    .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180);},{m_elbow}))
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;}))
    .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); },{m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (3.5);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(90);},{m_elbow}))
    .AlongWith(frc2::cmd::RunOnce([=]{ return m_elbow->getWristAngle()>85.5;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0);},{m_elbow}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(0); m_elbow->setElbowAngle(255);}, {m_elbow}))
    .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180);},{m_elbow}))
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;}))
    .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); },{m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (3.5);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(-0.25);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0.25);}))
    )
    
}
