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


using namespace DriveConstants;

RobotContainer::RobotContainer()
{
   // autoChooser = AutoBuilder::buildAutoChooser(); 

    DriverCommands(&m_drive, &m_driverController, &m_secondaryController);


    autoChooser = AutoBuilder::buildAutoChooser(); 
    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}
 
frc2::Command* RobotContainer::GetAutonomousCommand()
{
     return autoChooser.GetSelected();
}