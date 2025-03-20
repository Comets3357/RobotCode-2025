#include "RobotContainer.h"
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <utility>
#include <frc2/command/FunctionalCommand.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h> 


using namespace pathplanner;


// This will start Redux CANLink manually for C++

#include "commands/IntakeCommands.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
    OperatorCommands(&m_drive, &m_climb, &m_elevator, &m_elbow, &m_intake, &m_LED, &m_driverController, &m_secondaryController, offset);
    DriverCommands(&m_drive, &m_climb, &m_elevator, &m_elbow, &m_intake, &m_LED, &m_driverController, &m_secondaryController);
    AutonCommands(&m_drive, &m_climb, &m_elevator, &m_elbow, &m_intake, &m_LED);

    autoChooser = AutoBuilder::buildAutoChooser(); 
    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}
 
frc2::Command* RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected();
}