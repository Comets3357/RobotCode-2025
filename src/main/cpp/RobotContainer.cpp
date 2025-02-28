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

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <memory>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>


using namespace pathplanner;


// This will start Redux CANLink manually for C++

#include "commands/IntakeCommands.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here
    // Configure the button bindings
    ConfigureButtonBindings();

    NamedCommands::registerCommand("Algae Start", std::move(IntakeAlgae(&intake)));
    NamedCommands::registerCommand("Algae Stop", std::move(StopIntake(&intake)));
    NamedCommands::registerCommand("Algae Up", std::move(StopDeploy(&intake)));

    NamedCommands::registerCommand("Starting Reset", std::move(
        frc2::cmd::RunOnce([this] {m_elevator.setPosition(8)}, {&m_elevator})
        .AlongWith(frc2::cmd::WaitUntil( [this] {return m_elevator.getAPosiiton() > 7;}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setElbowAngle(90)}, {&m_elbowSubsystem}))
        .AlongWith(frc2::cmd::WaitUntil( [this] {return m_elbow.getElbowAngle() > 80}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elevator.setPosition(4)}, {&m_elevator}))
    ));

    NamedCommands::registerCommand("L4", std::move(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((50));},{&m_elevator})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (49.5);}))
         .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);},{&m_elbowSubsystem}
    ))));

    NamedCommands::registerCommand("Place L4", std::move(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32)); m_elbowSubsystem.setRollerSpeed(-0.3); },{&m_elbowSubsystem, &m_elevator})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (32.5);})))
         .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); },{&m_elbowSubsystem})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;})))
         .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}
    ))); 

    NamedCommands::registerCommand("Aim L1", std::move(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(0); m_elbowSubsystem.setElbowAngle(255);}, {&m_elbowSubsystem})
        .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()>254;})
    )));
    
    NamedCommands::registerCommand("Score L1", std::move(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setRollerSpeed(-0.25);})
        .AlongWith(frc2::cmd::Wait(units::second_t{0.3}))
    ));

    NamedCommands::registerCommand("Reset L1", std::move(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(90);},{&m_elbowSubsystem}))
        .AlongWith(frc2::cmd::WaitUntil([this]{ return m_elbowSubsystem.getWristAngle()>85.5;}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(180);},{&m_elbowSubsystem})
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0);},{&m_elbowSubsystem}
    ))));

    NamedCommands::registerCommand("Intake Piece", std::move(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(180); m_elbowSubsystem.setElbowAngle(235); }, {&m_elbowSubsystem})
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()>234;}))
          .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0.25);},{&m_elbowSubsystem}))));

    NamedCommands::registerCommand("Stop Intake Piece", std::move(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0); m_elbowSubsystem.setWristAngle(90); m_elbowSubsystem.setElbowAngle(180); }, {&m_elbowSubsystem})
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<181;}))));

    
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

        // ZERO GYRO BUTTON

        m_driverController.Start().OnTrue(frc2::cmd::RunOnce([this] {m_drive.ZeroHeading();}, {&m_drive})); 
   
        //intake down
        m_secondaryController.RightTrigger().OnTrue( frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(295); m_elbowSubsystem.setWristAngle(0); m_elbowSubsystem.setRollerSpeed(0.25);}, {&m_elbowSubsystem})
        .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getWristAngle() < 2;}))
        .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(305);},{&m_elbowSubsystem}))
        .AlongWith(frc2::cmd::WaitUntil([this] {return m_elbowSubsystem.getRollerCurrent() > 50;}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0);},{&m_elbowSubsystem})));

        //intake up
        m_secondaryController.A().OnFalse(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0.2);},{&m_elbowSubsystem})
        .AlongWith(frc2::cmd::WaitUntil([this]{return m_elbowSubsystem.getElbowAngle()<=295;}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(90);},{&m_elbowSubsystem}))
        .AlongWith(frc2::cmd::RunOnce([this]{ return m_elbowSubsystem.getWristAngle()>85.5;}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0.25);}, {&m_elbowSubsystem}))
        .AlongWith(frc2::cmd::Wait(units::second_t{1}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0);},{&m_elbowSubsystem})));

        m_secondaryController.POVUp().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((50));},{&m_elevator})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (49.5);}))
         .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);},{&m_elbowSubsystem}))
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
         .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32)); m_elbowSubsystem.setRollerSpeed(-0.1); },{&m_elbowSubsystem, &m_elevator})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (32.5);})))
         .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); },{&m_elbowSubsystem})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;})))
         .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}))); 

         m_secondaryController.POVLeft().OnTrue(frc2::cmd::RunOnce([this] { m_elevator.setPosition(17);}, {&m_elevator})
         .AlongWith(frc2::cmd::WaitUntil([this]{ return m_elevator.getAPosition()>16.5;}))
         .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(225);},{&m_elevator}))
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
          .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(250); m_elbowSubsystem.setRollerSpeed(-0.15);},{&m_elbowSubsystem})));

          m_secondaryController.POVRight().OnTrue(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(225);}, {&m_elbowSubsystem})
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
          .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(250); m_elbowSubsystem.setRollerSpeed(-0.15);},{&m_elbowSubsystem})));

           m_secondaryController.POVDown().OnTrue(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(0); m_elbowSubsystem.setElbowAngle(255);}, {&m_elbowSubsystem})
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
          .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(-0.25);},{&m_elbowSubsystem})));

        
          m_secondaryController.X().OnTrue(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180);},{&m_elbowSubsystem})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;}))
         .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}))
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (3.5);}))
         .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(90);},{&m_elbowSubsystem}))
         .AlongWith(frc2::cmd::RunOnce([this]{ return m_elbowSubsystem.getWristAngle()>85.5;}))
         .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0.5);},{&m_elbowSubsystem}))
         .AlongWith(frc2::cmd::Wait(units::second_t{0.25}))
         .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0);},{&m_elbowSubsystem})));

          m_secondaryController.B().OnTrue(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(180); m_elbowSubsystem.setElbowAngle(235); }, {&m_elbowSubsystem})
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()>234;}))
          .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0.25);},{&m_elbowSubsystem})));

         m_secondaryController.B().OnFalse(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0); m_elbowSubsystem.setWristAngle(90); m_elbowSubsystem.setElbowAngle(180); }, {&m_elbowSubsystem})
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<181;})));

          m_driverController.LeftTrigger().OnTrue(IntakeAlgae(&intake));
          m_driverController.LeftTrigger().OnFalse(StopIntake(&intake));

          m_driverController.LeftBumper().OnTrue(DeployAlgae(&intake));
          m_driverController.LeftBumper().OnFalse(StopDeploy(&intake));

        
        // m_driverController.A().OnTrue(IntakeAlgae(&intake));
        // m_driverController.A().OnFalse(StopIntake(&intake));   
}
 
frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
     return PathPlannerAuto("3 Piece").ToPtr();
}

void RobotContainer::ConfigureButtonBindings() {}
