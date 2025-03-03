#include "Subsystems/ClimbSubsystem.h"
#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/ElbowSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/LEDSubsystem.h"
#include "Subsystems/MAXSwerveModule.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "commands/IntakeCommands.h"
                                                                    
void DriverCommands::DriverCommands() {

    //  _____       _                  ____        _   _                  
    // |  __ \     (_)                |  _ \      | | | |                 
    // | |  | |_ __ ___   _____ _ __  | |_) |_   _| |_| |_ ___  _ __  ___ 
    // | |  | | '__| \ \ / / _ \ '__| |  _ <| | | | __| __/ _ \| '_ \/ __|
    // | |__| | |  | |\ V /  __/ |    | |_) | |_| | |_| || (_) | | | \__ \
    // |_____/|_|  |_| \_/ \___|_|    |____/ \__,_|\__|\__\___/|_| |_|___/
                                                                    

    //BUTTONS FOR DRIVING
    //Zeroes the gyro for driving convenience
    m_driverController.Start().OnTrue(frc2::cmd::RunOnce([this] {m_drive.ZeroHeading();}, {&m_drive})); 

    //Functions to drive the swerve modules, adds a conditional for speed reduction.
    m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {

        if (!halfSpeed) {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true);
        }
        else 
        {
            //divides the values from the joysticks by 3 to reduce the speed.
            m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband) / 3.0 },
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband) / 3.0 },
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband) / 3.0 },
            true);
        }
    },
    {&m_drive}));

    //RIGHT TRIGGER
    //Halves the speed of swerve 
    m_driverController.RightTrigger().OnTrue(frc2::cmd::RunOnce([this] {halfSpeed = true;})); 
    m_driverController.RightTrigger().OnFalse(frc2::cmd::RunOnce([this] {halfSpeed = false;}));

    //   ____  _   _                 ____        _   _                  
    //  / __ \| | | |               |  _ \      | | | |                 
    // | |  | | |_| |__   ___ _ __  | |_) |_   _| |_| |_ ___  _ __  ___ 
    // | |  | | __| '_ \ / _ \ '__| |  _ <| | | | __| __/ _ \| '_ \/ __|
    // | |__| | |_| | | |  __/ |    | |_) | |_| | |_| || (_) | | | \__ \
    //  \____/ \__|_| |_|\___|_|    |____/ \__,_|\__|\__\___/|_| |_|___/

    //LEFT TRIGGER
    //Deploys the algae subsystem to pick up an algae ball
    m_driverController.LeftTrigger().OnTrue(IntakeAlgae(&intake));
    m_driverController.LeftTrigger().OnFalse(StopIntake(&intake));

    //LEFT BUMPER
    //Deploys an algae ball already inside the bot
    //Puts the algae subsystem down and spins the rollers in reverse
    m_driverController.LeftBumper().OnTrue(DeployAlgae(&intake));
    m_driverController.LeftBumper().OnFalse(StopDeploy(&intake));

    

}