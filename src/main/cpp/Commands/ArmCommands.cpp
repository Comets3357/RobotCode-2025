#include "Commands/ArmCommands.h"

frc2::CommandPtr WristRotateLeft(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, double idle)
{

    return frc2::FunctionalCommand([=]{},
        [=]{m_elbow->setElbowAngle(idle + (m_secondaryController->GetRightY() * 20)); 
        if (m_driverController->GetHID().GetLeftTriggerAxis() > 0.5) {
            m_elbow->WristRotate();
        }
        if (m_elbow->getWristAngle()<45 || m_elbow->getWristAngle()>315)
            {
             m_elbow->setWristAngle(270);
            }
        if (m_elbow->getWristAngle()<225 && m_elbow->getWristAngle()>135)
            {
             m_elbow->setWristAngle(90);
            }},
    [=](bool interrupt){},
    [=](){return m_secondaryController->GetHID().GetRightBumperButton();},{m_elbow}).ToPtr();
}

frc2::CommandPtr WristRotateRight(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, double idle)
{

    return frc2::FunctionalCommand([=]{},
        [=]{m_elbow->setElbowAngle(idle + (-m_secondaryController->GetRightY() * 20)); 
        if (m_driverController->GetHID().GetLeftTriggerAxis() > 0.5) {
            m_elbow->WristRotate();
        }
        if (m_elbow->getWristAngle()<45 || m_elbow->getWristAngle()>315)
            {
             m_elbow->setWristAngle(270);
            }
        if (m_elbow->getWristAngle()<225 && m_elbow->getWristAngle()>135)
            {
             m_elbow->setWristAngle(90);
            }},
    [=](bool interrupt){},
    [=](){return m_secondaryController->GetHID().GetRightBumperButton();},{m_elbow}).ToPtr();
}

frc2::CommandPtr lowLevelWristRotate(ElbowSubsystem* m_elbow, frc2::CommandXboxController* m_driverController, frc2::CommandXboxController* m_secondaryController, double idle) {

    return frc2::FunctionalCommand([=]{},
        [=]{m_elbow->setElbowAngle(idle + (-m_secondaryController->GetRightY() * 7)); },
    [=](bool interrupt){},
    [=](){return m_secondaryController->GetHID().GetRightBumperButton();},{m_elbow}).ToPtr();
}