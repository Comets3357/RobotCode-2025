#include "Commands/ArmCommands.h"
#include "Subsystems/ElbowSubsystem.h"

//Forces wrist rotation to rotate to the left, takes in the different subsystems for usage within the function itself
frc2::CommandPtr wristRotateLeft(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, double idle)
{

    return frc2::FunctionalCommand([=]{},
        [=]{m_elbow->setElbowAngle(idle + (m_secondaryController->GetRightY() * 20)); 
        if (m_driverController->GetHID().GetLeftTriggerAxis() > 0.5) {
            m_elbow->WristRotate();
        }

        if (m_elbow->getWristAngle() < 45 || m_elbow->getWristAngle() > 315) {
            m_elbow->setWristAngle(270);
        }

        if (m_elbow->getWristAngle() < 225 && m_elbow->getWristAngle() > 135) {
            m_elbow->setWristAngle(90);
        }},

    [=](bool interrupt){},
    [=](){return m_secondaryController->GetHID().GetRightBumperButton();},{m_elbow}).ToPtr()
    .HandleInterrupt([this]{});
}

//forces wrist rotation to rotate to the right, takes in diff. subsystems for the function
frc2::CommandPtr wristRotateRight(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, double idle)
{

    return frc2::FunctionalCommand([=]{},
        [=]{m_elbow->setElbowAngle(idle + (-m_secondaryController->GetRightY() * 20)); 
        if (m_driverController->GetHID().GetLeftTriggerAxis() > 0.5) {
            m_elbow->WristRotate();
        }

        if (m_elbow->getWristAngle()<45 || m_elbow->getWristAngle()>315) {
            m_elbow->setWristAngle(270);
        }

        if (m_elbow->getWristAngle()<225 && m_elbow->getWristAngle()>135) {
            m_elbow->setWristAngle(90);
        }},

    [=](bool interrupt){},
    [=](){return m_secondaryController->GetHID().GetRightBumperButton();},{m_elbow}).ToPtr()
    .HandleInterrupt([this]
    {
        int pov = POVCheck(frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController)

        if (pov = 1 || pov = 2 || pov =3 || pov = 4)
        {
            m_elbow->setWristAngle(95); 
        }
    });
}

double POVCheck(frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController) {
    double latest = m_secondaryController->GetHID().GetPOV();

    if (latest == 0)  
        return 1; 
    if (latest == 90)  
        return 2; 
    if (latest == 180)  
        return 3;  
    if (latest == 270)  
        return 4;    

    return 0;  // Return 0 if no valid POV direction is pressed
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



