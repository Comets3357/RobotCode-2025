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

void OperatorCommands::OperatorCommands(DriveSubsystem* m_drive, ClimbSubsystem* m_climb, ElevatorSubsystem* m_elevator,
                    ElbowSubsystem* m_elbow, IntakeSubsystem* m_intake, LEDSubsystem* m_LED) {

    
    //               _   _               ____        _   _                  
    //     /\       | | (_)             |  _ \      | | | |                 
    //    /  \   ___| |_ _  ___  _ __   | |_) |_   _| |_| |_ ___  _ __  ___ 
    //   / /\ \ / __| __| |/ _ \| '_ \  |  _ <| | | | __| __/ _ \| '_ \/ __|
    //  / ____ \ (__| |_| | (_) | | | | | |_) | |_| | |_| || (_) | | | \__ |
    // /_/    \_\___|\__|_|\___/|_| |_| |____/ \__,_|\__|\__\___/|_| |_|___/
                                                                      
    //m_intake to ground (arm side)
    m_secondaryController.A().OnTrue(frc2::cmd::RunOnce([this] {m_elbow.setElbowAngle(295); m_elbow.setWristAngle(0); m_elbow.setRollerSpeed(0.4);}, {&m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbow.getWristAngle() < 2;}))
    .AndThen(frc2::cmd::RunOnce([this]{m_elbow.setElbowAngle(305);},{&m_elbow}))
    );

    //m_intake up to idle position
    m_secondaryController.A().OnFalse(frc2::cmd::RunOnce([this] {m_elbow.setElbowAngle(180); m_elbow.setRollerSpeed(0.2);},{&m_elbow})
    .AlongWith(frc2::cmd::WaitUntil([this]{return m_elbow.getElbowAngle()<=295;}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setWristAngle(90);},{&m_elbow}))
    .AlongWith(frc2::cmd::RunOnce([this]{ return m_elbow.getWristAngle()>85.5;}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setRollerSpeed(0.25);}, {&m_elbow}))
    .AlongWith(frc2::cmd::Wait(units::second_t{1}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setRollerSpeed(0);},{&m_elbow}))
    );

    //reset all positions to idle mode, elevator all down and elbow set to 180.
    m_secondaryController.X().OnTrue(frc2::cmd::RunOnce([this]{m_elbow.setElbowAngle(180);},{&m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbow.getElbowAngle()<=185;}))
    .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (3.5);}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setWristAngle(90);},{&m_elbow}))
    .AlongWith(frc2::cmd::RunOnce([this]{ return m_elbow.getWristAngle()>85.5;}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setRollerSpeed(0.5);},{&m_elbow}))
    .AlongWith(frc2::cmd::Wait(units::second_t{0.25}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setRollerSpeed(0);},{&m_elbow})));

    //
    m_secondaryController.B().OnTrue(frc2::cmd::RunOnce([this] {m_elbow.setWristAngle(180); m_elbow.setElbowAngle(235); }, {&m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbow.getElbowAngle()>234;}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setRollerSpeed(0.4);},{&m_elbow})));

    m_secondaryController.B().OnFalse(frc2::cmd::RunOnce([this] {m_elbow.setRollerSpeed(0); m_elbow.setWristAngle(90); m_elbow.setElbowAngle(180); }, {&m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbow.getElbowAngle()<181;})));

    m_secondaryController.Y().OnTrue(IntakeAlgae(&m_intake)); 
    m_secondaryController.Y().OnFalse(StopIntake(&m_intake));

    //  _____   ______      __  ____        _   _                  
    // |  __ \ / __ \ \    / / |  _ \      | | | |                 
    // | |__) | |  | \ \  / /  | |_) |_   _| |_| |_ ___  _ __  ___ 
    // |  ___/| |  | |\ \/ /   |  _ <| | | | __| __/ _ \| '_ \/ __|
    // | |    | |__| | \  /    | |_) | |_| | |_| || (_) | | | \__ |
    // |_|     \____/   \/     |____/ \__,_|\__|\__\___/|_| |_|___/

    //Moves elevator to L4 position and if A pressed, flip the wrist 180 degrees
    //Then move elbow down to score
    //TODO make this not suck (reformat functional command)
    
    // frc2::FunctionalCommand([this]{m_elbow.setElbowAngle(240);}, 
    // [this]{m_elbow.setElbowAngle(225 + (m_secondaryController.getRightY()*15))}, [this](bool interrupt){}, 
    // [this]{}, {&m_elbow}).ToPtr();

    // frc2::FunctionalCommand([this](){m_elbow.setElbowAngle(240);}, 
    // [this](){ if (m_driverController.GetHID().GetAButtonPressed()){offset += 180; m_elbow.setWristAngle(offset);}}, [this](bool interrupt){}, 
    // [this](){ return m_secondaryController.GetHID().GetRightBumperButton(); }, {&m_elbow, &m_elevator}).ToPtr()

//     frc2::FunctionalCommand(
//     [this]{m_elbow.setElbowAngle(240);},
//     [this]{m_elbow.setElbowAngle(225 + (m_secondaryController.getRightY() * 15)); if (m_driverController.GetHID().GetAButtonPressed()) {
//             offset += 180; m_elbow.setWristAngle(offset);}},[this](bool interrupt){},[this](){return m_secondaryController.GetHID().GetRightBumperButton();},{&m_elbow, &m_elevator}
// ).ToPtr();

    m_secondaryController.POVUp().OnTrue( frc2::cmd::RunOnce([this] { m_elevator.setPosition(50); m_elbow.setElbowAngle(240);}, {&m_elevator})
    .AlongWith(frc2::cmd::WaitUntil([this]{ return m_elevator.getAPosition()>49.5;}))
    .AndThen(frc2::FunctionalCommand([this]{},
    [this]{m_elbow.setElbowAngle(225 + (m_secondaryController.getRightY() * 15)); if (m_driverController.GetHID().GetAButtonPressed()) {
    offset += 180; m_elbow.setWristAngle(offset);}},[this](bool interrupt){},[this](){return m_secondaryController.GetHID().GetRightBumperButton();},{&m_elbow, &m_elevator}).ToPtr())
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setElbowAngle(270); m_elbow.setRollerSpeed(-0.15);},{&m_elbow})));

    //Moves elevator the L3 position
    //If right trigger is pressed elbow goes down to score.
    m_secondaryController.POVLeft().OnTrue(frc2::cmd::RunOnce([this] { m_elevator.setPosition(17);}, {&m_elevator})
    .AlongWith(frc2::cmd::WaitUntil([this]{ return m_elevator.getAPosition()>16.5;}))
    .AndThen(frc2::cmd::RunOnce([this]{m_elbow.setElbowAngle(225);},{&m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setElbowAngle(250); m_elbow.setRollerSpeed(-0.15);},{&m_elbow}))
    );

    //Moves elevator the L2 position
    //If right trigger is pressed elbow goes down to score.
    m_secondaryController.POVRight().OnTrue(frc2::cmd::RunOnce([this] {m_elbow.setElbowAngle(225);}, {&m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setElbowAngle(250); m_elbow.setRollerSpeed(-0.15);},{&m_elbow}))
    );

    //Moves elbow parallel to ground
    //If right trigger is pressed rollers score.
    m_secondaryController.POVDown().OnTrue(frc2::cmd::RunOnce([this] {m_elbow.setWristAngle(0); m_elbow.setElbowAngle(255);}, {&m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [this] { return m_secondaryController.GetHID().GetRightBumperButton();}))
    .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setRollerSpeed(-0.25);},{&m_elbow}))
    );


                       

    // START / BACK BUTTON CONTROLS

    //Moves the bot to climb position to prevent subsystem clash, once everything is
    //in position, it moves the climb out
    //TODO MAKE CLIMB NOT TERRIBLE
    m_secondaryController.Start().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((20));},{&m_elevator})
     .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (19.5);}))
     .AndThen(frc2::cmd::RunOnce([this] {m_elbow.setElbowAngle(240);},{&m_elbow}))
     .AlongWith(frc2::cmd::WaitUntil([this] {return m_elbow.getElbowAngle() > 235;}))
     .AndThen(frc2::cmd::RunOnce([this] {m_climb.ClimbSetPercent(-0.3);}, {&m_climb}))
    ); 

    //On start false stop moving the climb
    m_secondaryController.Start().OnFalse(frc2::cmd::RunOnce([this]{m_climb.ClimbSetPercent(0);},{&m_climb})); 
       
    //On Back true slowly recline the climb to get off the ground, on false stop the climb.
    m_secondaryController.Back().OnTrue(frc2::cmd::RunOnce([this] {m_climb.ClimbSetPercent(0.3);}, {&m_climb}));
    m_secondaryController.Back().OnFalse(frc2::cmd::RunOnce( [this] {m_climb.ClimbSetPercent(0);}, {&m_climb}));


}
