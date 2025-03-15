#include "Commands/ArmCommands.h"


//Forces wrist rotation to rotate to the left, takes in the different subsystems for usage within the function itself
frc2::CommandPtr wristRotateLeft(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, double idle)
{

    return frc2::FunctionalCommand([=]{},
        [=]{m_elbow->setElbowAngle(idle + (m_secondaryController->GetRightY() * 20)); 
        if (m_driverController->GetHID().GetAButtonPressed()) {
            m_elbow->placementRotate();
        }

        if (m_elbow->getWristAngle() < 45 || m_elbow->getWristAngle() > 315) {
            m_elbow->setWristAngle(270);
        }

        if (m_elbow->getWristAngle() < 225 && m_elbow->getWristAngle() > 135) {
            m_elbow->setWristAngle(90);
        }},

    [=](bool interrupt){},
    [=](){return m_secondaryController->GetHID().GetRightBumperButton();},{m_elbow}).ToPtr();
}

//forces wrist rotation to rotate to the right, takes in diff. subsystems for the function
frc2::CommandPtr wristRotateRight(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, double idle)
{

    return frc2::FunctionalCommand([=]{},
        [=]{m_elbow->setElbowAngle(idle + (-m_secondaryController->GetRightY() * 20)); 
        if (m_driverController->GetHID().GetAButtonPressed()) {
            m_elbow->placementRotate();
        }

        if (m_elbow->getWristAngle()<45 || m_elbow->getWristAngle()>315) {
            m_elbow->setWristAngle(270);
        }

        if (m_elbow->getWristAngle()<225 && m_elbow->getWristAngle()>135) {
            m_elbow->setWristAngle(90);
        }},

    [=](bool interrupt){},
    [=](){return m_secondaryController->GetHID().GetRightBumperButton();},{m_elbow}).ToPtr();
}

frc2::CommandPtr autonWristRotation(ElbowSubsystem* m_elbow, double idle)
{

    return frc2::FunctionalCommand([=]{},
        [=]{m_elbow->setElbowAngle(idle); 

        if (m_elbow->getWristAngle() < 45 || m_elbow->getWristAngle() > 225) {
            m_elbow->setWristAngle(180);
        }

        if (m_elbow->getWristAngle()< 135 && m_elbow->getWristAngle()> 45) {
            m_elbow->setWristAngle(0);
        }},

    [=](bool interrupt){},
    [=](){return m_secondaryController->GetHID().GetRightBumperButton();},{m_elbow}).ToPtr();
}



