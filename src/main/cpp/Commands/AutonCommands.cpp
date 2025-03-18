

#include "Subsystems/ClimbSubsystem.h"
#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/ElbowSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/LEDSubsystem.h"
#include "Subsystems/MAXSwerveModule.h"
#include "Commands/ArmCommands.h"
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/Commands.h>
#include "RobotContainer.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include "commands/IntakeCommands.h"

using namespace pathplanner;
                                                                    
void AutonCommands(DriveSubsystem* m_drive, ClimbSubsystem* m_climb, ElevatorSubsystem* m_elevator,
                    ElbowSubsystem* m_elbow, IntakeSubsystem* m_intake, LEDSubsystem* m_LED) 
{

    // vision positions // 
    const frc::Pose2d pose1{13.65_m, 5.32_m, frc::Rotation2d{150_deg}}; 
    const auto MOE = 0.03_m; 
    const auto MOErotation = 1.5_deg; 
    const units::time::second_t bufferTime{5.0};

    NamedCommands::registerCommand("Algae Start", std::move(IntakeAlgae(m_intake)));
    NamedCommands::registerCommand("Algae Stop", std::move(StopIntake(m_intake)));
    NamedCommands::registerCommand("Algae Up", std::move(StopDeploy(m_intake)));

    // NamedCommands::registerCommand("Wrist Flip Check", std::move(( frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(0);}, {m_elbow})
    // .AlongWith(frc2::cmd::WaitUntil([=] {return (m_elbow->getWristAngle() < 5) || (m_elbow->getWristAngle() > 358);})))
    // .AndThen((frc2::cmd::RunOnce([=] { m_elbow->setSideOne(m_elbow->getDistanceMeasurement()); }))
    // .AlongWith(frc2::cmd::RunOnce([=] {m_elbow->detectionRotate();}, {m_elbow}))
    // .AlongWith(frc2::cmd::WaitUntil([=] {return (m_elbow->getWristAngle() > 60) && (m_elbow->getWristAngle() < 120);}))
    // .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(180);}, {m_elbow}))
    // .AlongWith(frc2::cmd::WaitUntil([=] {return (m_elbow->getWristAngle() > 175) && (m_elbow->getWristAngle() < 185);})))
    // .AndThen(frc2::cmd::RunOnce([=] { m_elbow->setSideTwo(m_elbow->getDistanceMeasurement()); }))
    // .AndThen(frc2::cmd::Either(frc2::cmd::RunOnce( [=] {m_elbow->setWristAngle(180);}, {m_elbow}), 
    // frc2::cmd::RunOnce([=] {m_elbow->detectionRotate();}, {m_elbow})
    //     .AlongWith(frc2::cmd::WaitUntil([=] {return (m_elbow->getWristAngle() > 60) && (m_elbow->getWristAngle() < 120);}))
    //     .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(180);}, {m_elbow})),
    //     [=] {return !m_elbow->isAutonWristFlipValid();})
    // )));

    NamedCommands::registerCommand("Starting Reset", std::move( frc2::cmd::RunOnce([=] {m_elevator->setPosition(25);}, {m_elevator})
    .AlongWith(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0.1);}, {m_elbow}))
    .AlongWith(frc2::cmd::WaitUntil( [=] {return m_elevator->getAPosition() > 24;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(190);}, {m_elbow}))
    .AlongWith(frc2::cmd::WaitUntil( [=] {return m_elbow->getElbowAngle() > 265;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elevator->setPosition(4); m_elbow->setRollerSpeed(0);}, {m_elevator, m_elbow})))
    );

    NamedCommands::registerCommand("L4", std::move(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((50));}, {m_elevator})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() > (49.5);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(240);}, {m_elbow})))
    );

    NamedCommands::registerCommand("Place L4", std::move(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((32)); m_elbow->setRollerSpeed(-0.3); }, {m_elevator, m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (32.5);})))
    .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180); m_elbow->setRollerSpeed(0); }, {m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;}))
    .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); }, {m_elevator})))
    ); 

    NamedCommands::registerCommand("Aim L1", std::move(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(0); m_elbow->setElbowAngle(255);}, {m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()>254;})
    )));
    
    NamedCommands::registerCommand("Score L1", std::move(frc2::cmd::RunOnce([=]{m_elbow->setRollerSpeed(-0.15);}, {m_elbow})
    .AlongWith(frc2::cmd::Wait(units::second_t{0.3}))
    ));

    NamedCommands::registerCommand("Reset L1", std::move(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(90);}, {m_elbow}))
    .AlongWith(frc2::cmd::WaitUntil([=]{ return m_elbow->getWristAngle()>85.5;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(180);}, {m_elbow})
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0);}, {m_elbow})))
    );

    NamedCommands::registerCommand("Intake Piece", std::move(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(180); m_elbow->setElbowAngle(235); }, {m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()>234;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0.25);}, {m_elbow})))
    );

    NamedCommands::registerCommand("Stop Intake Piece", std::move(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0); m_elbow->setWristAngle(90); m_elbow->setElbowAngle(180); })
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<181;})))
    );

    NamedCommands::registerCommand("Attempt", std::move(frc2::cmd::Run([=]{m_drive->GoToPos(pose1);})
    .RaceWith(frc2::cmd::Wait(bufferTime))
    .RaceWith(frc2::cmd::WaitUntil([=]{return m_drive->inRange(m_drive->GetPose(), pose1, MOE, MOErotation);}))
    .AndThen(frc2::cmd::RunOnce([=]{m_drive->Drive(0_mps, 0_mps, units::radians_per_second_t{0}, true);}))
    .AndThen(
        frc2::cmd::Either(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((50));})                              // if true it runs the l4 aim and score sequence
            .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() > (49.5);}))
            .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(260);}))
            .AlongWith(frc2::cmd::WaitUntil([=] {return m_elbow->getElbowAngle() > 250;}))
            .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((32)); m_elbow->setRollerSpeed(-0.3); })
            .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (32.5);})))
            .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180); m_elbow->setRollerSpeed(0); })
            .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;})))
            .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); })), 
    
            frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(0); m_elbow->setElbowAngle(255);})       // if false it will run l1 score and aim sequence
            .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()>254;}))
            .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setRollerSpeed(-0.15);})
            .AlongWith(frc2::cmd::Wait(units::second_t{0.3}))), 
    
    [=]{return m_drive->inRange(m_drive->GetPose(), pose1, MOE, MOErotation);}))));                        // condition for which command to run

    //Either
    //If In Range and winning the race group: do the targeting for L4 and attempt place
    //If In Range and losing the race group: Place L1


}

