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

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here
    // Configure the button bindings
    ConfigureButtonBindings();

    NamedCommands::registerCommand("Algae Start", std::move(IntakeAlgae(&intake)));
    NamedCommands::registerCommand("Algae Stop", std::move(StopIntake(&intake)));
    NamedCommands::registerCommand("Algae Up", std::move(StopDeploy(&intake)));

    NamedCommands::registerCommand("L4", std::move(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((50));},{&m_elevator})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (49.5);}))
         .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);},{&m_elbowSubsystem}
    ))));

    NamedCommands::registerCommand("Place L4", std::move(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32)); m_elbowSubsystem.setRollerSpeed(-0.1); },{&m_elbowSubsystem, &m_elevator})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (32.5);})))
         .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); },{&m_elbowSubsystem})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;})))
         .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}
    ))); 

    NamedCommands::registerCommand("Aim L1", std::move(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(0); m_elbowSubsystem.setElbowAngle(255);}, {&m_elbowSubsystem})
        .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()>254;})
    )));
    
    NamedCommands::registerCommand("Score L1", std::move(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setRollerSpeed(-0.25);})
        .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (3.5);}))
    ));

    NamedCommands::registerCommand("Reset L1", std::move(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(90);},{&m_elbowSubsystem}))
        .AlongWith(frc2::cmd::WaitUntil([this]{ return m_elbowSubsystem.getWristAngle()>85.5;}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(180);},{&m_elbowSubsystem})
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0);},{&m_elbowSubsystem}
    ))));

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
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
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand(){
    
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
}

void RobotContainer::ConfigureBindings()
{
    
}

void RobotContainer::ConfigureButtonBindings() {}
