#include "RobotContainer.h"
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
#include <iostream>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <memory>

#include <memory>


using namespace pathplanner;


// This will start Redux CANLink manually for C++
#include "commands/IntakeCommands.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here

    /*
            Initialize all namecommands from pathplanner here
            Put constants for position in the constant folder
    */
    // NamedCommands::registerCommand("ElevatorL4", std::move(frc2::cmd::RunOnce([this]
    //                                                       {m_elevator.setPosition(PositionConstats::elevatorL4pos);},{&m_elevator}))); 
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

    // Since we are using a holonomic drivetrain, the rotation component of this pose
// represents the goal holonomic rotation

}


frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
     return PathPlannerAuto("New Auto").ToPtr();
}

void RobotContainer::ConfigureButtonBindings() {

    //drive
    m_driverController.LeftBumper().WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
    m_driverController.X().WhileTrue(new frc2::RunCommand([this] { m_drive.ZeroHeading(); }, {&m_drive})); 
    m_driverController.A().WhileTrue(pathfindingCommand.get());
    //algae intake
    // m_driverController.RightBumper().OnTrue(IntakeAlgae(&intake));
    // m_driverController.RightBumper().OnFalse(StopIntake(&intake));

    // m_driverController.RightTrigger().OnTrue(DeployAlgae(&intake));
    // m_driverController.RightTrigger().OnFalse(StopDeploy(&intake));

    //elevator
    // m_driverController.A().OnTrue(frc2::cmd::RunOnce([this]
    //                                                       {m_elevator.setPosition(0.5);},{&m_elevator}));
    // m_driverController.B().OnTrue(frc2::cmd::RunOnce([this]
    //                                                     {m_elevator.setPosition(20);},{&m_elevator}));
 
}

