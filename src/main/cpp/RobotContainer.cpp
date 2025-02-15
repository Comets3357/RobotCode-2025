#include "RobotContainer.h"

#include "Commands/DefaultElbowCommand.h"
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "Subsystems/ElbowSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();

    // Set up default drive command
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this]
        {
            m_drive.Drive(
                -units::meters_per_second_t{frc::ApplyDeadband(
                    m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
                -units::meters_per_second_t{frc::ApplyDeadband(
                    m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
                -units::radians_per_second_t{frc::ApplyDeadband(
                    m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
                true);
        },
        {&m_drive}));

    m_elbowSubsystem.SetDefaultCommand(DefaultElbowCommand(&m_elbowSubsystem, 
    [this] { return m_driverController.GetRightY(); },
    [this] { return m_driverController.GetRightTriggerAxis(); }
    ).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return frc2::cmd::Print("No Autonomous Command!");
}

void RobotContainer::ConfigureButtonBindings() {

    //elbow stuff
    m_driverController.Y().OnTrue(DefaultElbowCommand::setGripperPos(&m_elbowSubsystem, 0));
    m_driverController.RightBumper().OnTrue(DefaultElbowCommand::setGripperPos(&m_elbowSubsystem, 90));

    m_driverController.A().OnTrue(DefaultElbowCommand::setIdle(&m_elbowSubsystem));
    m_driverController.B().OnTrue(DefaultElbowCommand::setIntake(&m_elbowSubsystem));
    m_driverController.X().OnTrue(DefaultElbowCommand::setOuttake(&m_elbowSubsystem));
    
    //drive
    // m_driverController.LeftBumper().WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
    // m_driverController.X().WhileTrue(new frc2::RunCommand([this] { m_drive.ZeroHeading(); }, {&m_drive})); 

    // //algae intake
    // m_driverController.RightBumper().OnTrue(IntakeAlgae(&intake));
    // m_driverController.RightBumper().OnFalse(StopIntake(&intake));

    // m_driverController.RightTrigger().OnTrue(DeployAlgae(&intake));
    // m_driverController.RightTrigger().OnFalse(StopDeploy(&intake));

    // //elevator
    // m_driverController.A().OnTrue(frc2::cmd::RunOnce([this]
    //                                                       {m_elevator.setPosition(0.5);},{&m_elevator}));
    // m_driverController.B().OnTrue(frc2::cmd::RunOnce([this]
    //                                                     {m_elevator.setPosition(20);},{&m_elevator}));
 
}

