#include "Commands/AutonCommands.h"
#include "Constants.h"

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

    NamedCommands::registerCommand("Starting Reset", std::move( frc2::cmd::RunOnce([=] {m_elevator->setPosition(25);})
    .AlongWith(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0.1);}))
    .AlongWith(frc2::cmd::WaitUntil( [=] {return m_elevator->getAPosition() > 24;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(190);}))
    .AlongWith(frc2::cmd::WaitUntil( [=] {return m_elbow->getElbowAngle() > 265;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elevator->setPosition(4); m_elbow->setRollerSpeed(0);})))
    );

    NamedCommands::registerCommand("L4", std::move(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((PositionConstats::elevatorL4pos));})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() > (PositionConstats::elevatorL4pos - 0.5);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(240);})))
    );

    NamedCommands::registerCommand("Place L4", std::move(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((32)); m_elbow->setRollerSpeed(-0.3); })
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (32.5);})))
    .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180); m_elbow->setRollerSpeed(0); })
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;})))
    .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(PositionConstats::elevatorRestPos); }))
    ); 

    NamedCommands::registerCommand("Aim L1", std::move(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(0); m_elbow->setElbowAngle(255);})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()>254;})
    )));
    
    NamedCommands::registerCommand("Score L1", std::move(frc2::cmd::RunOnce([=]{m_elbow->setRollerSpeed(-0.15);})
    .AlongWith(frc2::cmd::Wait(units::second_t{0.3}))
    ));

    NamedCommands::registerCommand("Reset L1", std::move(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(90);}))
    .AlongWith(frc2::cmd::WaitUntil([=]{ return m_elbow->getWristAngle()>85.5;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(180);})
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0);})))
    );

    NamedCommands::registerCommand("Intake Piece", std::move(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(180); m_elbow->setElbowAngle(235); })
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()>234;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0.25);})))
    );

    NamedCommands::registerCommand("Stop Intake Piece", std::move(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0); m_elbow->setWristAngle(90); m_elbow->setElbowAngle(180); })
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<181;}))));

    NamedCommands::registerCommand("Attempt", std::move(frc2::cmd::Run([=]{m_drive->GoToPos(pose1);})
    .RaceWith(frc2::cmd::Wait(bufferTime))
    .RaceWith(frc2::cmd::WaitUntil([=]{return m_drive->inRange(m_drive->GetPose(), pose1, MOE, MOErotation);}))
    .AndThen(frc2::cmd::RunOnce([=]{m_drive->Drive(0_mps, 0_mps, units::radians_per_second_t{0}, true);}))
    .AndThen(
        frc2::cmd::Either(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((PositionConstats::elevatorL4pos));})                              // if true it runs the l4 aim and score sequence
            .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() > (PositionConstats::elevatorL4pos - 0.5);}))
            .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(260);}))
            .AlongWith(frc2::cmd::WaitUntil([=] {return m_elbow->getElbowAngle() > 250;}))
            .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((32)); m_elbow->setRollerSpeed(-0.3); })
            .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (32.5);})))
            .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180); m_elbow->setRollerSpeed(0); })
            .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;})))
            .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(PositionConstats::elevatorRestPos); })), 
    
            frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(0); m_elbow->setElbowAngle(255);})       // if false it will run l1 score and aim sequence
            .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()>254;}))
            .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setRollerSpeed(-0.15);})
            .AlongWith(frc2::cmd::Wait(units::second_t{0.3}))), 
    
    [=]{return m_drive->inRange(m_drive->GetPose(), pose1, MOE, MOErotation);}))));                        // condition for which command to run

    //Either
    //If In Range and winning the race group: do the targeting for L4 and attempt place
    //If In Range and losing the race group: Place L1


}

