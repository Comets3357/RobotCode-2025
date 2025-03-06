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
#include <frc2/command/FunctionalCommand.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h


using namespace pathplanner;


// This will start Redux CANLink manually for C++

#include "commands/IntakeCommands.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
    autoChooser = AutoBuilder::buildAutoChooser(); 
    // Initialize all of your commands and subsystems here
    // Configure the button bindings
    ConfigureButtonBindings();
     frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

    // auto chooser builder // 
      
     // frc::SmartDashboard::PutData("Auto Chooser", &autoChooser); 


    NamedCommands::registerCommand("Algae Start", std::move(IntakeAlgae(&m_intake)));
    NamedCommands::registerCommand("Algae Stop", std::move(StopIntake(&m_intake)));
    NamedCommands::registerCommand("Algae Up", std::move(StopDeploy(&m_intake)));

    NamedCommands::registerCommand("Starting Reset", std::move( frc2::cmd::RunOnce([this] {m_elevator.setPosition(25);}, {&m_elevator})
        .AlongWith(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0.1);}, {&m_elbowSubsystem}))
        .AlongWith(frc2::cmd::WaitUntil( [this] {return m_elevator.getAPosition() > 24;}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(190);}, {&m_elbowSubsystem}))
        .AlongWith(frc2::cmd::WaitUntil( [this] {return m_elbowSubsystem.getElbowAngle() > 265;}))
        .AndThen(frc2::cmd::RunOnce([this] {m_elevator.setPosition(4); m_elbowSubsystem.setRollerSpeed(0);}, {&m_elevator, &m_elbowSubsystem}))
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
    
    NamedCommands::registerCommand("Score L1", std::move(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setRollerSpeed(-0.15);})
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

    
    
}
 
frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
     return PathPlannerAuto("3 Piece").ToPtr();//std::nullptr_t;//autoChooser.GetSelected();
}



void RobotContainer::ConfigureButtonBindings() {}
