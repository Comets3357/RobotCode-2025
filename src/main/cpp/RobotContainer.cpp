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

//...

// This will start Redux CANLink manually for C++

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
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::cmd::Print("No Autonomous Command!");
}

void RobotContainer::ConfigureButtonBindings()
{
    m_driverController.RightBumper().WhileTrue(new frc2::RunCommand([this]
                                                                    { m_drive.SetX(); }, {&m_drive}));
    m_driverController.X().WhileTrue(new frc2::RunCommand([this]
                                                          { m_drive.ZeroHeading(); }, {&m_drive}));
        // make go 
    m_driverController.A().OnTrue(new frc2::RunCommand([this]{ 
        double test = testspeed; 
        m_drive.Drive(units::meters_per_second_t{testspeed}, 
                    units::meters_per_second_t{0},  
                    units::radians_per_second_t{0}, 
                    true);}, {&m_drive})); 

    m_driverController.A().OnFalse(new frc2::RunCommand([this]{ 
     
        m_drive.Drive(units::meters_per_second_t{0}, 
                    units::meters_per_second_t{0},  
                    units::radians_per_second_t{0}, 
                    true);}, {&m_drive}));
    // make go in reverse 
    m_driverController.B().OnTrue(new frc2::RunCommand([this]{ 
   
        m_drive.Drive(units::meters_per_second_t{testspeed}, 
                    units::meters_per_second_t{0},  
                    units::radians_per_second_t{0}, 
                    true);}, {&m_drive})); 

    m_driverController.B().OnFalse(new frc2::RunCommand([this]{ 
        double test = 0;
        m_drive.Drive(units::meters_per_second_t{test}, 
                    units::meters_per_second_t{test},  
                    units::radians_per_second_t{0}, 
                    true);}, {&m_drive}));
}

void RobotContainer::ConfigureBindings()
{
}
