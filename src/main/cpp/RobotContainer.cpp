#include "RobotContainer.h"

#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc2/command/Commands.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "RobotContainer.h"
#include "subsystems/ClimbSubsystem.h"
#include <Commands/ClimbCommand.h>

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

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

      climb.SetDefaultCommand(frc2::RunCommand(
        [this] {
            climb.ClimbSetPercent(m_driverController.GetRightY() * .25);
        }, {&climb}));

}

void RobotContainer::ConfigureBindings() {
    ClimbSubsystem* climb;
  // Map the A button to extend the elevator
  m_driverController.A().WhileTrue(ClimbStop(climb));
 
  // Map the B button to retract the elevator
  m_driverController.B().WhileTrue(ClimbRetract(climb));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand(){
    
}

void RobotContainer::ConfigureButtonBindings() {
//   frc2::JoystickButton(&m_driverController,
//                        frc::XboxController::Button::kRightBumper)
//       .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
}



