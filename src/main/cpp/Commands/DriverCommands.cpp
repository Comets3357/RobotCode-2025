#include "Subsystems/ClimbSubsystem.h"
#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/ElbowSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/LEDSubsystem.h"
#include "Subsystems/MAXSwerveModule.h"
#include <frc2/command/button/CommandXboxController.h>

#include <frc2/command/Commands.h>
#include "RobotContainer.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "commands/IntakeCommands.h"
#include <frc/DriverStation.h>
                                                                    
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

   m_driverController->A().WhileTrue(frc2::cmd::Run([=] {m_drive->GoToPos(m_drive->TestingPointRed);}, {m_drive})); 
   m_driverController->B().OnTrue(frc2::cmd::RunOnce([=] {m_drive->UpdateNonVisionPose();}, {m_drive}));
    //m_driverController->A().OnFalse(frc2::cmd::RunOnce([=] {m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, true);}, {m_drive})); 

   // m_driverController->B().WhileTrue(frc2::cmd::Run([=] {m_drive->GoToPos((frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) ? m_drive->TopLeftBlue : m_drive->TopLeftRed);}, {m_drive})); 
    //m_driverController->B().OnFalse(frc2::cmd::RunOnce([=] {m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, true);}, {m_drive})); 

    m_driverController->POVUp().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetY += 0.02_m;}));
    m_driverController->POVDown().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetY -= 0.02_m;}));
    m_driverController->POVLeft().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetX -= 0.02_m;}));
    m_driverController->POVRight().OnTrue(frc2::cmd::RunOnce([=] {m_drive->visionPoseOffsetX += 0.02_m;}));
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
    m_driverController->RightBumper().WhileTrue(rotateTo(m_drive, 144_deg, m_driverController));

    //LEFT BUMPER
    //Auto Aligns robot to a certain angle
    //preferably used to align robot to human player station
    m_driverController->LeftBumper().WhileTrue(rotateTo(m_drive, 36_deg, m_driverController));

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