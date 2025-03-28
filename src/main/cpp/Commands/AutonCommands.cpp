#include "Commands/AutonCommands.h"





using namespace pathplanner;
                                                                    
void AutonCommands(DriveSubsystem* m_drive, ClimbSubsystem* m_climb, ElevatorSubsystem* m_elevator,
                    ElbowSubsystem* m_elbow, IntakeSubsystem* m_intake, LEDSubsystem* m_LED) 
{


    NamedCommands::registerCommand("Algae Start", std::move(IntakeAlgae(m_intake)));
    NamedCommands::registerCommand("Algae Stop", std::move(StopIntake(m_intake)));
    NamedCommands::registerCommand("Algae Up", std::move(StopDeploy(m_intake)));

    NamedCommands::registerCommand("Starting Reset", std::move( 
        
        frc2::cmd::RunOnce([=] {m_elevator->setPosition(19.2);})
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0.4);m_elbow->setWristAngle(40);m_elbow->setElbowAngle(340);}, {m_elbow})) 
    .AndThen(frc2::cmd::WaitUntil([=]{return m_elbow->getWristAngle() < 42 && m_elbow->getElbowAngle() < 342;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(0);m_elbow->setElbowAngle(330);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elevator->setPosition(25);}))

    .AlongWith(frc2::cmd::WaitUntil( [=] {return m_elevator->getAPosition() > 24;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(190);}))
    .AlongWith(frc2::cmd::WaitUntil( [=] {return m_elbow->getElbowAngle() > 265;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elevator->setPosition(4); m_elbow->setRollerSpeed(0);})))
    .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setWristAngle(90);}))
    );

    NamedCommands::registerCommand("L4", std::move(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((50));})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() > (49.5);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(240);})))
    );

    NamedCommands::registerCommand("Place L4", std::move(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((32)); m_elbow->setRollerSpeed(-0.3); })
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (32.5);})))
    .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180); m_elbow->setRollerSpeed(0); })
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;})))
    .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); }))
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

    NamedCommands::registerCommand("Attempt L4 Sequence", std::move(BetterGoToScore(m_drive->left9, m_drive, m_elbow, m_elevator))/*GoToAndScore((frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) ? m_drive->TopLeftBlue : m_drive->TopLeftRed, m_drive, m_elbow, m_elevator))*/); 
    NamedCommands::registerCommand("Human Player Score Reef", std::move(BetterGoToScore(m_drive->left9, m_drive, m_elbow, m_elevator))); 
    NamedCommands::registerCommand("left9 Score", std::move(BetterGoToScore(m_drive->left9, m_drive, m_elbow, m_elevator))); 
}

frc2::CommandPtr GoToAndScore(frc::Pose2d targetPose, DriveSubsystem* m_drive, ElbowSubsystem* m_elbow, ElevatorSubsystem* m_elevator,
    units::second_t bufferTime, units::meter_t MOE,  units::degree_t MOErotation)
{
    return 
        frc2::cmd::Run([=]
        {   
        })
     
        .RaceWith(frc2::cmd::WaitUntil([=]{return true /*m_drive->inRange(m_drive->GetPose(), targetPose, MOE, MOErotation)*/;}))
        .RaceWith(frc2::cmd::Wait(bufferTime))
       // .AndThen(frc2::cmd::RunOnce([=]{m_drive->Drive(0_mps, 0_mps, units::radians_per_second_t{0}, true);}))
        .AndThen(
            frc2::cmd::Either(
                frc2::cmd::RunOnce([=]{ m_elevator->setPosition((50));})                              // if true it runs the l4 aim and score sequence
                .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() > (49.5);}))
                .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(100);}))
                .AlongWith(frc2::cmd::WaitUntil([=] {return m_elbow->getElbowAngle() < 105;}))
                .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((32)); m_elbow->setRollerSpeed(-0.3); })
                .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (32.5);})))
                .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180); m_elbow->setRollerSpeed(0); })
                .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;})))
                .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); }))
                .AlongWith(frc2::cmd::WaitUntil([=] {return m_elevator->getAPosition() < 7;}))
                //.AndThen(frc2::cmd::RunOnce([=]{m_drive->Drive(0_mps, 0_mps, units::radians_per_second_t{0}, true);})) // will stop drive after placing
                , 
        
                frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(0); m_elbow->setElbowAngle(255);})       // if false it will run l1 score and aim sequence
                .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()>254;}))
                .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setRollerSpeed(-0.15);})
                .AlongWith(frc2::cmd::Wait(units::second_t{0.3})))
                .AndThen(frc2::cmd::RunOnce([=]{m_drive->Drive(0_mps, 0_mps, units::radians_per_second_t{0}, true);})) // will stop the drive after placing
                ,


        
        [=]{return m_drive->inRange(m_drive->GetPose(), targetPose, MOE, MOErotation);}));
       // .RaceWith();
}

frc2::CommandPtr BetterGoToScore(frc::Pose2d targetPose, DriveSubsystem* m_drive, ElbowSubsystem* m_elbow, ElevatorSubsystem* m_elevator, units::meter_t MOE,  units::degree_t MOErotation)
    {
        return 
        frc2::cmd::Run([=]{m_drive->GoToPos(targetPose);})
     
        .RaceWith(frc2::cmd::WaitUntil([=]{return m_drive->inRange(m_drive->GetPose(), targetPose, MOE, MOErotation);})
        .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((50));}))                              // if true it runs the l4 aim and score sequence
        .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() > (49.5) && m_drive->inRange(m_drive->GetPose(), targetPose);}))
        .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(100);}))
        .AlongWith(frc2::cmd::WaitUntil([=] {return m_elbow->getElbowAngle() < 105;}))
        .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((32)); m_elbow->setRollerSpeed(-0.3); })
        .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (32.5);})))
        .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180); m_elbow->setRollerSpeed(0); })
        .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;})))
        .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); }))
        .AlongWith(frc2::cmd::WaitUntil([=] {return m_elevator->getAPosition() < 7;})));
    }


