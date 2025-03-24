#include "Subsystems/ClimbSubsystem.h"
#include "Subsystems/DriveSubsystem.h"
#include "Subsystems/ElbowSubsystem.h"
#include "Subsystems/ElevatorSubsystem.h"
#include "Subsystems/IntakeSubsystem.h"
#include "Subsystems/LEDSubsystem.h"
#include "Subsystems/MAXSwerveModule.h"
#include <frc2/command/button/CommandXboxController.h>

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "commands/IntakeCommands.h"
#include "commands/ArmCommands.h"


//   ____                       _               ____        _   _                  
//  / __ \                     | |             |  _ \      | | | |                 
// | |  | |_ __   ___ _ __ __ _| |_ ___  _ __  | |_) |_   _| |_| |_ ___  _ __  ___ 
// | |  | | '_ \ / _ \ '__/ _` | __/ _ \| '__| |  _ <| | | | __| __/ _ \| '_ \/ __|
// | |__| | |_) |  __/ | | (_| | || (_) | |    | |_) | |_| | |_| || (_) | | | \__ |
//  \____/| .__/ \___|_|  \__,_|\__\___/|_|    |____/ \__,_|\__|\__\___/|_| |_|___/
//        | |                                                                      
//        |_|   

void OperatorCommands(DriveSubsystem* m_drive, ClimbSubsystem* m_climb, ElevatorSubsystem* m_elevator,
                    ElbowSubsystem* m_elbow, IntakeSubsystem* m_intake, LEDSubsystem* m_LED, 
                    frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, int &offset) {

    
    //               _   _               ____        _   _                  
    //     /\       | | (_)             |  _ \      | | | |                 
    //    /  \   ___| |_ _  ___  _ __   | |_) |_   _| |_| |_ ___  _ __  ___ 
    //   / /\ \ / __| __| |/ _ \| '_ \  |  _ <| | | | __| __/ _ \| '_ \/ __|
    //  / ____ \ (__| |_| | (_) | | | | | |_) | |_| | |_| || (_) | | | \__ |
    // /_/    \_\___|\__|_|\___/|_| |_| |____/ \__,_|\__|\__\___/|_| |_|___/
                                                                      
    //m_intake to ground (arm side)
    m_secondaryController->A().OnTrue(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(295); m_elbow->setWristAngle(0); m_elbow->setRollerSpeed(0.4); m_elevator->setPosition(3);}, {m_elbow, m_elevator})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return (m_elbow->getWristAngle() < 2) && (m_elevator->getAPosition() < 5);}))
    .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(305);}, {m_elbow}))
    .AlongWith(frc2::cmd::WaitUntil([=] {return (m_elbow->isGamePieceDetected() == true) && (m_elbow->getElbowAngle() > 285);}))
    .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setRollerSpeed(0);}, {m_elbow}))
    );

    //m_intake up to idle position
    m_secondaryController->A().OnFalse(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(180); m_elbow->setRollerSpeed(0.2);},{m_elbow})
    .AlongWith(frc2::cmd::WaitUntil([=]{return m_elbow->getElbowAngle()<=295;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(90);},{m_elbow}))
    .AlongWith(frc2::cmd::RunOnce([=]{ return m_elbow->getWristAngle()>85.5;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0.25);},{m_elbow}))
    .AlongWith(frc2::cmd::Wait(units::second_t{1}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0);},{m_elbow}))
    );

    m_secondaryController->B().OnTrue(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(180); m_elbow->setElbowAngle(235);},{m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()>234;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0.4);},{m_elbow}))
    .AlongWith(frc2::cmd::WaitUntil([=] {return (m_elbow->isGamePieceDetected() == true);}))
    .AndThen(frc2::cmd::RunOnce([=]{m_elbow->setRollerSpeed(0);}, {m_elbow}))
    );

    m_secondaryController->B().OnFalse(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0); m_elbow->setWristAngle(90); m_elbow->setElbowAngle(180);},{m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<181;})));

    //reset all positions to idle mode, elevator all down and elbow set to 180.
    m_secondaryController->X().OnTrue(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180);},{m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;}))
    .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); },{m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (3.5);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(90);},{m_elbow}))
    .AlongWith(frc2::cmd::RunOnce([=]{ return m_elbow->getWristAngle()>85.5;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0.5);},{m_elbow}))
    .AlongWith(frc2::cmd::Wait(units::second_t{0.25}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0);},{m_elbow})));

    //Intake and deploy algae
    (m_secondaryController->Y() && m_secondaryController->LeftBumper()).OnTrue(
        DeployAlgae(m_intake)
    );

    (m_secondaryController->Y() && !m_secondaryController->LeftBumper()).OnTrue(
        IntakeAlgae(m_intake)
    );
    
    (m_secondaryController->Y() && m_secondaryController->LeftBumper()).OnFalse(
        StopDeploy(m_intake)
    );
    
    (m_secondaryController->Y() && !m_secondaryController->LeftBumper()).OnFalse(
        StopIntake(m_intake)
    );

    // elevator zero sequence the new button will be "back" 

   // m_driverController->B().OnTrue(frc2::cmd::RunOnce([=] {m_elevator->setSpeed(-0.15);},{m_elevator}).IgnoringDisable(true)
    //.AlongWith(frc2::cmd::WaitUntil([=]{return m_elevator->ElevatorLimitPressed();}).IgnoringDisable(true))
    //.AndThen(frc2::cmd::RunOnce([=]{m_elevator->setSpeed(0); m_elevator->SetElevatorAbsolutePosition();},{m_elevator}).IgnoringDisable(true)));

    m_driverController->A().OnTrue(frc2::cmd::RunOnce([=] {m_LED->hPlayerGround = true;}));
    m_driverController->A().OnFalse(frc2::cmd::RunOnce([=] {m_LED->hPlayerGround = false;}));

     m_driverController->B().OnTrue(frc2::cmd::RunOnce([=] {m_LED->hPlayer = true;}));
    m_driverController->B().OnFalse(frc2::cmd::RunOnce([=] {m_LED->hPlayer = false;}));


    m_driverController->Back().OnTrue(frc2::cmd::RunOnce([=] {m_elevator->setSpeed(-0.15);},{m_elevator}).IgnoringDisable(true)
    .AlongWith(frc2::cmd::WaitUntil([=]{return m_elevator->ElevatorLimitPressed();}).IgnoringDisable(true))
    .AndThen(frc2::cmd::RunOnce([=]{m_elevator->setSpeed(0); m_elevator->SetElevatorAbsolutePosition();},{m_elevator}).IgnoringDisable(true)));

    //  _____   ______      __  ____        _   _                  
    // |  __ \ / __ \ \    / / |  _ \      | | | |                 
    // | |__) | |  | \ \  / /  | |_) |_   _| |_| |_ ___  _ __  ___ 
    // |  ___/| |  | |\ \/ /   |  _ <| | | | __| __/ _ \| '_ \/ __|
    // | |    | |__| | \  /    | |_) | |_| | |_| || (_) | | | \__ |
    // |_|     \____/   \/     |____/ \__,_|\__|\__\___/|_| |_|___/

    //Moves elevator to L4 position and if A pressed, flip the wrist 180 degrees
    //Then move elbow down to score

    // m_secondaryController->POVUp().OnTrue( frc2::cmd::RunOnce([=] { m_elevator->setPosition(50);})
    // .AlongWith(frc2::cmd::WaitUntil([=]{ return m_elevator->getAPosition()>49.5;}))
    // .AndThen(frc2::FunctionalCommand([=](){m_elbow->setElbowAngle(240);}, 
    // [=, &offset](){ if (m_driverController->GetHID().GetAButtonPressed()){offset += 180; m_elbow->setWristAngle(offset);}}, [=](bool interrupt){}, 
    // [=](){ return m_secondaryController->GetHID().GetRightBumperButton(); }).ToPtr())
    // .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(270); m_elbow->setRollerSpeed(-0.15);})));

     m_secondaryController->POVUp().OnTrue( frc2::cmd::RunOnce([=] { m_elevator->setPosition(50);}, {m_elevator})
     .AlongWith(frc2::cmd::WaitUntil([=]{ return m_elevator->getAPosition()>49.5;}))
     .AndThen(wristRotateLeft(m_elbow, m_driverController, m_secondaryController, 240))
     .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(270); m_elbow->setRollerSpeed(-0.15);},{m_elbow})));

      (m_secondaryController->POVUp() && m_secondaryController->LeftBumper()).OnTrue( frc2::cmd::RunOnce([=] { m_elevator->setPosition(50);}, {m_elevator})
     .AlongWith(frc2::cmd::WaitUntil([=]{ return m_elevator->getAPosition()>49.5;}))
     .AndThen(wristRotateRight(m_elbow, m_driverController, m_secondaryController, 120))
     .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(90); m_elbow->setRollerSpeed(-0.15);},{m_elbow})));

    m_secondaryController->RightBumper().OnFalse(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180);},{m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185 && m_elbow->getElbowAngle()>=175;}))
    .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); },{m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (3.5);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(90);},{m_elbow}))
    .AlongWith(frc2::cmd::RunOnce([=]{ return m_elbow->getWristAngle()>85.5;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0);},{m_elbow})));
        //  .AlongWith(frc2::cmd::Wait(units::second_t{0.25}))
        //  .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0);},{&m_elbow})));

    //Moves elevator the L3 position
    //If right trigger is pressed elbow goes down to score.
    m_secondaryController->POVLeft().OnTrue(frc2::cmd::RunOnce([=] { m_elevator->setPosition(17);}, { m_elevator})
    .AlongWith(frc2::cmd::WaitUntil([=]{ return m_elevator->getAPosition()>16.5;}))
    .AndThen(wristRotateLeft(m_elbow, m_driverController, m_secondaryController, 220))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(250); m_elbow->setRollerSpeed(-0.15);},{ m_elbow})));

    (m_secondaryController->POVLeft() && m_secondaryController->LeftBumper()).OnTrue(frc2::cmd::RunOnce([=] { m_elevator->setPosition(17);}, { m_elevator})
    .AlongWith(frc2::cmd::WaitUntil([=]{ return m_elevator->getAPosition()>16.5;}))
    .AndThen(wristRotateRight(m_elbow, m_driverController, m_secondaryController, 140))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(110); m_elbow->setRollerSpeed(-0.15);},{ m_elbow})));

    //Moves elevator the L2 position
    //If right trigger is pressed elbow goes down to score.
    m_secondaryController->POVRight().OnTrue(wristRotateLeft(m_elbow, m_driverController, m_secondaryController, 220)
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(250); m_elbow->setRollerSpeed(-0.15);},{ m_elbow})));
    m_secondaryController->POVRight().OnTrue(frc2::FunctionalCommand([=]{},
    [=, &offset]{m_elbow->setElbowAngle(225 + (m_secondaryController->GetRightY() * 15)); if (m_driverController->GetHID().GetAButtonPressed()) {
    offset += 180; m_elbow->setWristAngle(offset);}},[=](bool interrupt){},[=](){return m_secondaryController->GetHID().GetRightBumperButton();},{m_elbow}).ToPtr()
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(250); m_elbow->setRollerSpeed(-0.15);},{ m_elbow})));


    (m_secondaryController->POVRight() && m_secondaryController->LeftBumper()).OnTrue(wristRotateRight(m_elbow, m_driverController, m_secondaryController, 140)
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(110); m_elbow->setRollerSpeed(-0.15);},{ m_elbow})));

    //Moves elbow parallel to ground
    //If right trigger is pressed rollers score.
    m_secondaryController->POVDown().OnTrue(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(0); m_elbow->setElbowAngle(255);}, {m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_secondaryController->GetHID().GetRightBumperButton();}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(-0.25);}, {m_elbow}))
    );

    (m_secondaryController->POVDown() && m_secondaryController->LeftBumper()).OnTrue(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(0); m_elbow->setElbowAngle(105);})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_secondaryController->GetHID().GetRightBumperButton();}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(-0.25);}))
    );
                     
    // START / BACK BUTTON CONTROLS

    //Moves the bot to climb position to prevent subsystem clash, once everything is
    //in position, it moves the climb out
    //TODO MAKE CLIMB NOT TERRIBLE
    m_secondaryController->Start().OnTrue(frc2::cmd::RunOnce([=]{ m_elevator->setPosition((20));}, {m_elevator})
     .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() > (19.5);}))
     .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setElbowAngle(240);}, {m_elbow}))
     .AlongWith(frc2::cmd::WaitUntil([=] {return m_elbow->getElbowAngle() > 235;}))
     .AndThen(frc2::cmd::RunOnce([=] {m_climb->ClimbSetPercent(-0.3);}, {m_climb}))
    ); 

    //On start false stop moving the climb
    m_secondaryController->Start().OnFalse(frc2::cmd::RunOnce([=]{m_climb->ClimbSetPercent(0);}, {m_climb})); 
       
    //On Back true slowly recline the climb to get off the ground, on false stop the climb.
    m_secondaryController->Back().OnTrue(frc2::cmd::RunOnce([=] {m_climb->ClimbSetPercent(0.3);}, {m_climb}));
    m_secondaryController->Back().OnFalse(frc2::cmd::RunOnce( [=] {m_climb->ClimbSetPercent(0);}, {m_climb}));

    // OTHER BUTTONS

    //Run the rollers when left trigger is pressed, stop on false
    m_secondaryController->LeftTrigger().OnTrue(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0.20);}, {m_elbow}));
    m_secondaryController->LeftTrigger().OnFalse(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0);}, {m_elbow}));

    //Flip rollers 180 degrees
    m_secondaryController->RightTrigger().OnTrue(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle( m_elbow->getWristAngle() + 180);}, {m_elbow}));

    m_secondaryController->RightBumper().OnFalse(frc2::cmd::RunOnce([=]{m_elbow->setElbowAngle(180);},{m_elbow})
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elbow->getElbowAngle()<=185;}))
    .AndThen(frc2::cmd::RunOnce([=]{ m_elevator->setPosition(3); },{m_elevator}))
    .AlongWith(frc2::cmd::WaitUntil( [=] { return m_elevator->getAPosition() < (3.5);}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setWristAngle(90);},{m_elbow}))
    .AlongWith(frc2::cmd::RunOnce([=]{ return m_elbow->getWristAngle()>85.5;}))
    .AndThen(frc2::cmd::RunOnce([=] {m_elbow->setRollerSpeed(0);},{m_elbow})));


}
