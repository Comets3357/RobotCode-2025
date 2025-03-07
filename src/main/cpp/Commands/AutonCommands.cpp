

// #include "Subsystems/ClimbSubsystem.h"
// #include "Subsystems/DriveSubsystem.h"
// #include "Subsystems/ElbowSubsystem.h"
// #include "Subsystems/ElevatorSubsystem.h"
// #include "Subsystems/IntakeSubsystem.h"
// #include "Subsystems/LEDSubsystem.h"
// #include "Subsystems/MAXSwerveModule.h"
// #include <frc2/command/button/CommandXboxController.h>

// #include <frc2/command/Commands.h>
// #include "RobotContainer.h"

// #include <frc2/command/InstantCommand.h>
// #include <frc2/command/SequentialCommandGroup.h>
// #include "commands/IntakeCommands.h"
                                                                    
// void AutonCommands(DriveSubsystem* m_drive, ClimbSubsystem* m_climb, ElevatorSubsystem* m_elevator,
//                     ElbowSubsystem* m_elbow, IntakeSubsystem* m_intake, LEDSubsystem* m_LED) 
// {

//     NamedCommands::registerCommand("Algae Start", std::move(IntakeAlgae(m_intake)));
//     NamedCommands::registerCommand("Algae Stop", std::move(StopIntake(m_intake)));
//     NamedCommands::registerCommand("Algae Up", std::move(StopDeploy(m_intake)));

//     NamedCommands::registerCommand("Starting Reset", std::move( frc2::cmd::RunOnce([=] {m_elevator.setPosition(25);})
//         .AlongWith(frc2::cmd::RunOnce([=] {m_elbowSubsystem->setRollerSpeed(0.1);}))
//         .AlongWith(frc2::cmd::WaitUntil( [=] {return m_elevator->getAPosition() > 24;}))
//         .AndThen(frc2::cmd::RunOnce([=] {m_elbowSubsystem->setElbowAngle(190);}))
//         .AlongWith(frc2::cmd::WaitUntil( [=] {return m_elbowSubsystem->getElbowAngle() > 265;}))
//         .AndThen(frc2::cmd::RunOnce([=] {m_elevator->setPosition(4); m_elbowSubsystem->setRollerSpeed(0);}))
//     ));

//     NamedCommands::registerCommand("L4", std::move(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((50));})
//          .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() > (49.5);}))
//          .AndThen(frc2::cmd::RunOnce([=] {m_elbowSubsystem.setElbowAngle(240);}
//     ))));

//     NamedCommands::registerCommand("Place L4", std::move(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((32)); m_elbowSubsystem.setRollerSpeed(-0.3); })
//          .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (32.5);})))
//          .AndThen(frc2::cmd::RunOnce([=]{m_elbowSubsystem->setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); })
//          .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbowSubsystem->getElbowAngle()<=185;})))
//          .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); }
//     ))); 

//     NamedCommands::registerCommand("Aim L1", std::move(frc2::cmd::RunOnce([=] {m_elbowSubsystem->setWristAngle(0); m_elbowSubsystem.setElbowAngle(255);})
//         .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbowSubsystem->getElbowAngle()>254;})
//     )));
    
//     NamedCommands::registerCommand("Score L1", std::move(frc2::cmd::RunOnce([=]{m_elbowSubsystem->setRollerSpeed(-0.15);})
//         .AlongWith(frc2::cmd::Wait(units::second_t{0.3}))
//     ));

//     NamedCommands::registerCommand("Reset L1", std::move(frc2::cmd::RunOnce([=] {m_elbowSubsystem->setWristAngle(90);}))
//         .AlongWith(frc2::cmd::WaitUntil([=]{ return m_elbowSubsystem->getWristAngle()>85.5;}))
//         .AndThen(frc2::cmd::RunOnce([=] {m_elbowSubsystem->setElbowAngle(180);})
//         .AndThen(frc2::cmd::RunOnce([=] {m_elbowSubsystem->setRollerSpeed(0);}
//     ))));

//     NamedCommands::registerCommand("Intake Piece", std::move(frc2::cmd::RunOnce([=] {m_elbowSubsystem->setWristAngle(180); m_elbowSubsystem.setElbowAngle(235); })
//           .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbowSubsystem.getElbowAngle()>234;}))
//           .AndThen(frc2::cmd::RunOnce([=] {m_elbowSubsystem->setRollerSpeed(0.25);}))));

//     NamedCommands::registerCommand("Stop Intake Piece", std::move(frc2::cmd::RunOnce([=] {m_elbowSubsystem->setRollerSpeed(0); m_elbowSubsystem->setWristAngle(90); m_elbowSubsystem->setElbowAngle(180); })
//           .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbowSubsystem->getElbowAngle()<181;}))));

// }