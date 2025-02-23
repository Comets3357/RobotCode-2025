#include "RobotContainer.h"
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include "Commands/DefaultElbowCommand.h"
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <utility>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/ElbowSubsystem.h"
#include "commands/IntakeCommands.h"

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

    /*m_elbowSubsystem.SetDefaultCommand(DefaultElbowCommand(&m_elbowSubsystem, 
    [this] { return m_driverController.GetRightY(); },
    [this] { return m_driverController.GetRightTriggerAxis(); }
    ).ToPtr());*/

        // m_driverController.A().OnTrue(frc2::cmd::RunOnce([this]
        // { m_elevator.setPosition(20); }).AlongWith(frc2::cmd::WaitUntil([this]{ return m_elevator.getPosition() >= 19.8; }))
        // .AndThen(frc2::cmd::RunOnce([this]{m_elevator.setPosition(5);})).AlongWith(frc2::cmd::WaitUntil([this]{ return m_elevator.getPosition()<=5.2;})));

        // m_driverController.A().OnTrue( DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 90).AlongWith(frc2::cmd::RunOnce([this]{m_elevator.setPosition(35);})).AlongWith(frc2::cmd::WaitUntil([this]{ return m_elevator.getPosition() >= 19.8; })));
        //m_driverController.B().OnTrue(frc2::cmd::RunOnce([this]{m_elevator.setPosition(3);},{&m_elevator}).AlongWith(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 0)).AlongWith(frc2::cmd::WaitUntil([this]{return m_elevator.getPosition()<=5.2;})));

        // m_driverController.A().OnTrue(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 110).AlongWith(frc2::cmd::WaitUntil([this] {return m_elbowSubsystem.getElbowAngle()>=109;})));
        // //.AndThen(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0.2))/*.AlongWith(frc2::cmd::WaitUntil([this]{return m_elevator.getPosition()>=20;}))*/);
        //  m_driverController.A().OnTrue(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 50));
        //  m_driverController.B().OnTrue(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 110));
        //  m_driverController.X().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, -0.2));
        //  m_driverController.Y().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0.2));
        //  m_driverController.LeftBumper().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0));
        //  m_driverController.RightBumper().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 270));
        //  m_driverController.LeftStick().OnTrue(frc2::cmd::RunOnce([this]{m_elevator.setPosition(0.5);},{&m_elevator})); 
//m_driverController.B().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 90));        m_driverController.Y().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 180)); 
        
        // m_driverController.A().OnTrue((DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 295),
        //  DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 0))
        // .AlongWith(frc2::cmd::/* CONDITIONAL */WaitUntil( [this] { return m_elbowSubsystem.getWristAngle() < 2;}))
        // .AndThen(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0.3))
        // .AlongWith(frc2::cmd::/* CONDITIONAL */WaitUntil( [this] { return m_elbowSubsystem.getElbowSpeed() > 0.29;}))
        // .AndThen(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 305)));

            /**/

        // ZERO GYRO BUTTON

        //m_driverController.Start().OnTrue(frc2::cmd::RunOnce([this] {m_drive.ZeroHeading();}, {&m_drive})); 

        //intake down
         m_secondaryController.RightTrigger().OnTrue( frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(295); m_elbowSubsystem.setWristAngle(0); m_elbowSubsystem.setRollerSpeed(0.25);}, {&m_elbowSubsystem})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getWristAngle() < 2;}))
         .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(305);},{&m_elbowSubsystem})));


        //intake up
        m_secondaryController.RightTrigger().OnFalse(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0.2);},{&m_elbowSubsystem})
        .AlongWith(frc2::cmd::WaitUntil([this]{return m_elbowSubsystem.getElbowAngle()<=295;}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(90);},{&m_elbowSubsystem}))
        .AlongWith(frc2::cmd::RunOnce([this]{ return m_elbowSubsystem.getWristAngle()>85.5;}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0.25);}, {&m_elbowSubsystem}))
        .AlongWith(frc2::cmd::Wait(units::second_t{1}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0);},{&m_elbowSubsystem})));

        m_secondaryController.Y().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((50));},{&m_elevator})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (49.5);}))
         .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);},{&m_elbowSubsystem}))
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
         .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32)); m_elbowSubsystem.setRollerSpeed(-0.1); },{&m_elbowSubsystem, &m_elevator})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (32.5);})))
         .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); },{&m_elbowSubsystem})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;})))
         .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}))); 

        //  m_secondaryController.Y().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((51)); })
        //  .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (50);}))
        //  .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);})));

        //  m_secondaryController.X().OnTrue((frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32)); m_elbowSubsystem.setRollerSpeed(-0.1);}))
        //  .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (33);}))
        //  .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); }))
        //  .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;}))
        //  .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); })));
         
        // the correct one
        //  m_secondaryController.B().OnTrue((frc2::cmd::RunOnce([this]{ m_elevator.setPosition((51-25)); })
        //  .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (50-25);})))
        //  .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);})
        //  .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();})))
        //  .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32-25)); m_elbowSubsystem.setRollerSpeed(-0.1); })
        //  .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (32.5-25);})))
        //  .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); })
        //  .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;})))
        //  .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); })));

         m_secondaryController.B().OnTrue(frc2::cmd::RunOnce([this] { m_elevator.setPosition(17);}, {&m_elevator})
         .AlongWith(frc2::cmd::WaitUntil([this]{ return m_elevator.getAPosition()>16.5;}))
         .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(225);},{&m_elevator}))
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
          .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(250); m_elbowSubsystem.setRollerSpeed(-0.15);},{&m_elbowSubsystem})));

        //  m_secondaryController.B().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((51-25)); })
        //  .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (50-25);}))
        //  .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);})));

        //  m_secondaryController.A().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32-25));m_elbowSubsystem.setRollerSpeed(-0.1); })
        //  .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (32.5-25);}))
        //  .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); }))
        //  .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185; }))
        //  .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); })));



          m_secondaryController.X().OnTrue(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(225);}, {&m_elbowSubsystem})
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
          .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(250); m_elbowSubsystem.setRollerSpeed(-0.15);},{&m_elbowSubsystem})));

           m_secondaryController.A().OnTrue(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(0); m_elbowSubsystem.setElbowAngle(255);}, {&m_elbowSubsystem})
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
          .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(-0.25);},{&m_elbowSubsystem})));

        
          m_secondaryController.Start().OnTrue(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180);},{&m_elbowSubsystem})
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;}))
         .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}))
         .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (3.5);}))
         .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0.5);},{&m_elbowSubsystem}))
         .AlongWith(frc2::cmd::Wait(units::second_t{0.25}))
         .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0);},{&m_elbowSubsystem})));

          m_secondaryController.RightBumper().OnTrue(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(180); m_elbowSubsystem.setElbowAngle(235); }, {&m_elbowSubsystem})
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()>234;}))
          .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0.25);},{&m_elbowSubsystem})));

            m_secondaryController.RightBumper().OnFalse(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0); m_elbowSubsystem.setWristAngle(90); m_elbowSubsystem.setElbowAngle(180); }, {&m_elbowSubsystem})
          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<181;})));

          m_secondaryController.LeftBumper().OnTrue(IntakeAlgae(&intake));
          m_secondaryController.LeftBumper().OnFalse(StopIntake(&intake));

        //  m_secondaryController.LeftTrigger().OnTrue(DeployAlgae(&intake));
        //  m_secondaryController.LeftTrigger().OnFalse(StopDeploy(&intake));

         //m_driverController.B().OnTrue(frc2::cmd::RunOnce([this]{m_elevator.ClimbShot();}));   
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return frc2::cmd::Print("No Autonomous Command!");
}

void RobotContainer::ConfigureButtonBindings() {

        // m_driverController.LeftTrigger().OnTrue(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 295)
        // .AlongWith(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 0))
        // .AlongWith(frc2::cmd::/* CONDITIONAL */WaitUntil( [this] { return m_elbowSubsystem.getWristAngle() < 2}))
        // .AndThen(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 305)));




    //elbow
    // m_driverController.LeftBumper().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 0));

    // //pickup from ground
    // m_driverController.LeftTrigger().OnTrue(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 300));
    // m_driverController.LeftTrigger().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 0));
    // m_driverController.LeftTrigger().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0.4));
    // m_driverController.LeftTrigger().OnTrue(frc2::cmd::RunOnce([this] 
    //     {m_elevator.setPosition(4);}, {&m_elevator}));

    // //idle
    // m_driverController.RightTrigger().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0));
    // m_driverController.RightTrigger().OnTrue(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 180));
    // m_driverController.RightTrigger().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 90));
    // m_driverController.RightTrigger().OnTrue(frc2::cmd::RunOnce([this] 
    //     {m_elevator.setPosition(4);}, {&m_elevator}));

    // m_driverController.X().OnTrue(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 225));
    // m_driverController.X().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 0));
    // m_driverController.X().OnTrue(frc2::cmd::RunOnce([this] 
    //     {m_elevator.setPosition(10);}, {&m_elevator}));

    // m_driverController.B().OnTrue(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 225));
    // m_driverController.B().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 0));
    // m_driverController.B().OnTrue(frc2::cmd::RunOnce([this] 
    //     {m_elevator.setPosition(20);}, {&m_elevator}));

    // m_driverController.Y().OnTrue(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 210));
    // // m_driverController.Y().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 0));
    // // m_driverController.Y().OnTrue(frc2::cmd::RunOnce([this] 
    //     {m_elevator.setPosition(39);}, {&m_elevator}));

    // m_driverController.A().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 90));

    // m_driverController.RightBumper().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0.5));
    // m_driverController.RightBumper().OnFalse(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0));

    // m_driverController.LeftTrigger().OnTrue(DefaultElbowCommand::setElbowSpeed(&m_elbowSubsystem, 0.2));
    // m_driverController.LeftBumper().OnTrue(DefaultElbowCommand::setElbowSpeed(&m_elbowSubsystem, -0.2));
    

    // // m_driverController.X().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0.2));
    // // m_driverController.A().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0));
    // // m_driverController.B().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, -0.2));

    // m_driverController.X().OnTrue(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 110)); //ground state
    // m_driverController.X().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, -0.6)); 
    // m_driverController.X().OnTrue(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 90));

    // m_driverController.X().OnFalse(DefaultElbowCommand::setElbowPos(&m_elbowSubsystem, 40)); //idle state
    // m_driverController.X().OnFalse(DefaultElbowCommand::setWristPos(&m_elbowSubsystem, 180));
    // m_driverController.X().OnTrue(DefaultElbowCommand::setRollerSpeed(&m_elbowSubsystem, 0)); 


    //drive
    // m_driverController.LeftBumper().WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
    // m_driverController.X().WhileTrue(new frc2::RunCommand([this] { m_drive.ZeroHeading(); }, {&m_drive})); 

    //algae intake
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