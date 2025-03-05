#include "Subsystems/ClimbSubsystem.h"
#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/ElbowSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/LEDSubsystem.h"
#include "Subsystems/MAXSwerveModule.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "commands/IntakeCommands.h"

//   ____                       _               ____        _   _                  
//  / __ \                     | |             |  _ \      | | | |                 
// | |  | |_ __   ___ _ __ __ _| |_ ___  _ __  | |_) |_   _| |_| |_ ___  _ __  ___ 
// | |  | | '_ \ / _ \ '__/ _` | __/ _ \| '__| |  _ <| | | | __| __/ _ \| '_ \/ __|
// | |__| | |_) |  __/ | | (_| | || (_) | |    | |_) | |_| | |_| || (_) | | | \__ |
//  \____/| .__/ \___|_|  \__,_|\__\___/|_|    |____/ \__,_|\__|\__\___/|_| |_|___/
//        | |                                                                      
//        |_|   

void OperatorCommands::OperatorCommands() {

    
    //               _   _               ____        _   _                  
    //     /\       | | (_)             |  _ \      | | | |                 
    //    /  \   ___| |_ _  ___  _ __   | |_) |_   _| |_| |_ ___  _ __  ___ 
    //   / /\ \ / __| __| |/ _ \| '_ \  |  _ <| | | | __| __/ _ \| '_ \/ __|
    //  / ____ \ (__| |_| | (_) | | | | | |_) | |_| | |_| || (_) | | | \__ |
    // /_/    \_\___|\__|_|\___/|_| |_| |____/ \__,_|\__|\__\___/|_| |_|___/
                                                                      
    //intake down
    m_secondaryController.A().OnTrue(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(295); m_elbowSubsystem.setWristAngle(0); m_elbowSubsystem.setRollerSpeed(0.4);}, {&m_elbowSubsystem})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getWristAngle() < 2;}))
    .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(305);},{&m_elbowSubsystem}))
    );

    //intake up
    m_secondaryController.A().OnFalse(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0.2);},{&m_elbowSubsystem})
    .AlongWith(frc2::cmd::WaitUntil([this]{return m_elbowSubsystem.getElbowAngle()<=295;}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(90);},{&m_elbowSubsystem}))
    .AlongWith(frc2::cmd::RunOnce([this]{ return m_elbowSubsystem.getWristAngle()>85.5;}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0.25);}, {&m_elbowSubsystem}))
    .AlongWith(frc2::cmd::Wait(units::second_t{1}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(0);},{&m_elbowSubsystem}))
    );

    //  _____   ______      __  ____        _   _                  
    // |  __ \ / __ \ \    / / |  _ \      | | | |                 
    // | |__) | |  | \ \  / /  | |_) |_   _| |_| |_ ___  _ __  ___ 
    // |  ___/| |  | |\ \/ /   |  _ <| | | | __| __/ _ \| '_ \/ __|
    // | |    | |__| | \  /    | |_) | |_| | |_| || (_) | | | \__ |
    // |_|     \____/   \/     |____/ \__,_|\__|\__\___/|_| |_|___/

    m_secondaryController.POVUp().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((50));},{&m_elevator})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (49.5);}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);},{&m_elbowSubsystem}))
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
    .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32)); m_elbowSubsystem.setRollerSpeed(-0.1); },{&m_elbowSubsystem, &m_elevator})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (32.5);})))
    .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); },{&m_elbowSubsystem})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;})))
    .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}))
    ); 

    m_secondaryController.POVLeft().OnTrue(frc2::cmd::RunOnce([this] { m_elevator.setPosition(17);}, {&m_elevator})
    .AlongWith(frc2::cmd::WaitUntil([this]{ return m_elevator.getAPosition()>16.5;}))
    .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(225);},{&m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(250); m_elbowSubsystem.setRollerSpeed(-0.15);},{&m_elbowSubsystem}))
    );

    m_secondaryController.POVRight().OnTrue(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(225);}, {&m_elbowSubsystem})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(250); m_elbowSubsystem.setRollerSpeed(-0.15);},{&m_elbowSubsystem}))
    );

    m_secondaryController.POVDown().OnTrue(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setWristAngle(0); m_elbowSubsystem.setElbowAngle(255);}, {&m_elbowSubsystem})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setRollerSpeed(-0.25);},{&m_elbowSubsystem}))
    );


                       

    // START / BACK BUTTON CONTROLS

    //Moves the bot to climb position to prevent subsystem clash, once everything is
    //in position, it moves the climb out
    //TODO MAKE CLIMB NOT TERRIBLE
    m_secondaryController.Start().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((20));},{&m_elevator})
     .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (19.5);}))
     .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);},{&m_elbowSubsystem}))
     .AlongWith(frc2::cmd::WaitUntil([this] {return m_elbowSubsystem.getElbowAngle() > 235;}))
     .AndThen(frc2::cmd::RunOnce([this] {m_climb.ClimbSetPercent(-0.3);}, {&m_climb}))
    ); 

    //On start false stop moving the climb
    m_secondaryController.Start().OnFalse(frc2::cmd::RunOnce([this]{m_climb.ClimbSetPercent(0);},{&m_climb})); 
       
    //On Back true slowly recline the climb to get off the ground, on false stop the climb.
    m_secondaryController.Back().OnTrue(frc2::cmd::RunOnce([this] {m_climb.ClimbSetPercent(0.3);}, {&m_climb}));
    m_secondaryController.Back().OnFalse(frc2::cmd::RunOnce( [this] {m_climb.ClimbSetPercent(0);}, {&m_climb}));


}
