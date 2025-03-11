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

    OperatorCommands(&m_drive, &m_climb, &m_elevator, &m_elbow, &m_intake, &m_LED, &m_driverController, &m_secondaryController, offset);
    DriverCommands(&m_drive, &m_climb, &m_elevator, &m_elbow, &m_intake, &m_LED, &m_driverController, &m_secondaryController);
    AutonCommands(&m_drive, &m_climb, &m_elevator, &m_elbow, &m_intake, &m_LED);

    autoChooser = AutoBuilder::buildAutoChooser(); 
    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}
 




void RobotContainer::ConfigureButtonBindings() {}


// //         m_secondaryController.POVUp().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((50));},{&m_elevator})
//          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (49.5);}))
//          .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);},{&m_elbowSubsystem})));

//         frc2::FunctionalCommand cmd{[]{}, [this]{m_elevator.setPosition(50); m_elbowSubsystem.setElbowAngle(240);}, []{
//             frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32)); m_elbowSubsystem.setRollerSpeed(-0.1); },{&m_elbowSubsystem, &m_elevator})
//          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (32.5);}))
//          .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); },{&m_elbowSubsystem})
//          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;})))
//          .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}))
//         }, [this] {m_elevator.setPosition(m_elevator.getAPosition() - 0.5);}};


//          m_secondaryController.LeftBumper().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32)); m_elbowSubsystem.setRollerSpeed(-0.1); },{&m_elbowSubsystem, &m_elevator})
//          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (32.5);}))
//          .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); },{&m_elbowSubsystem})
//          .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;})))
//          .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}))); 

//          m_secondaryController.LeftTrigger().OnTrue(frc2::cmd::RunOnce([this] {m_elevator.setPosition(m_elevator.getAPosition() - 0.5);}));

        // m_driverController.A().OnTrue(IntakeAlgae(&intake));
        // m_driverController.A().OnFalse(StopIntake(&intake));   

 
frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
     return PathPlannerAuto("3 Piece").ToPtr();//std::nullptr_t;//autoChooser.GetSelected();
}



