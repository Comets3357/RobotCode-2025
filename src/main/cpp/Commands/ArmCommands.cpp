#include "Commands/ArmCommands.h"
#include "Subsystems/ElbowSubsystem.h"

//Forces wrist rotation to rotate to the left, takes in the different subsystems for usage within the function itself
frc2::CommandPtr wristRotateLeft(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, double idle, double scoreElbowPos)
{

    return frc2::FunctionalCommand([=]{},
        [=]{m_elbow->setElbowAngle(idle + (m_secondaryController->GetRightY() * 20)); 
        if (m_driverController->GetHID().GetLeftTriggerAxis() > 0.5) {
            m_elbow->WristRotate();
        }
        if (m_secondaryController->GetHID().GetLeftTriggerAxis() > 0.2) {
            m_elbow->setRollerSpeed(0.2);
        }

        if (m_elbow->getWristAngle() < 45 || m_elbow->getWristAngle() > 315) {
            m_elbow->setWristAngle(270);
        }

        if (m_elbow->getWristAngle() < 225 && m_elbow->getWristAngle() > 135) {
            m_elbow->setWristAngle(90);
        }},

    [=](bool interrupt){
    auto cmd = frc2::cmd::RunOnce([=] { 
        m_elbow->setElbowAngle(scoreElbowPos); 
        m_elbow->setRollerSpeed(-0.15); 
    }, {m_elbow});
    cmd.Schedule();
    },
    [=](){return m_secondaryController->GetHID().GetRightBumperButton();},{m_elbow}).ToPtr();
}

//forces wrist rotation to rotate to the right, takes in diff. subsystems for the function
frc2::CommandPtr wristRotateRight(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, double idle, double scoreElbowPos)
{

    return frc2::FunctionalCommand([=]{},
        [=]{
            bool primaryLT = false;
            bool secondaryLT = false;
            
            
            m_elbow->setElbowAngle(idle + (-m_secondaryController->GetRightY() * 20)); 
        if (m_driverController->GetHID().GetLeftTriggerAxis() > 0.5) {
            m_elbow->WristRotate();
            primaryLT = true;
        }
        if (m_secondaryController->GetHID().GetLeftTriggerAxis() > 0.2) {
            m_elbow->setRollerSpeed(0.2);
        }

        if (m_elbow->getWristAngle()<45 || m_elbow->getWristAngle()>315) {
            m_elbow->setWristAngle(270);
        }

        if (m_elbow->getWristAngle()<225 && m_elbow->getWristAngle()>135) {
            m_elbow->setWristAngle(90);
        }
        if(m_secondaryController->GetHID().GetLeftTriggerAxis() > 0.5) {
            m_elbow->setWristSpeed(0.2);
            secondaryLT = true;
        }
        frc::SmartDashboard::SmartDashboard::PutBoolean("Primary LT", primaryLT);
        frc::SmartDashboard::SmartDashboard::PutBoolean("Secondary LT", secondaryLT);
        },

    [=](bool interrupt){
    auto cmd = frc2::cmd::RunOnce([=] { 
        m_elbow->setElbowAngle(scoreElbowPos); 
        m_elbow->setRollerSpeed(-0.15); 
    }, {m_elbow});
    cmd.Schedule();
    },
    [=](){
        bool secondaryRB = false;
        if (m_secondaryController->GetHID().GetRightBumperButton()) {
            secondaryRB = true;
        }
        frc::SmartDashboard::SmartDashboard::PutBoolean("Secondary RB", secondaryRB);
        return secondaryRB;},{m_elbow}).ToPtr();
}

// frc2::CommandPtr autonWristRotation(ElbowSubsystem* m_elbow, double idle)
// {

//     return frc2::cmd::RunOnce([=] {

//     m_elbow->setElbowAngle(idle); 

//     if (m_elbow->getWristAngle() < 45 || m_elbow->getWristAngle() > 225) {
//         m_elbow->setWristAngle(180);
//     }

//     if (m_elbow->getWristAngle()< 135 && m_elbow->getWristAngle()> 45) {
//          m_elbow->setWristAngle(0);
//     }
    
//     }, {m_elbow});
// }



