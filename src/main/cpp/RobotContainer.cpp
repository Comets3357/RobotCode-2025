#include "RobotContainer.h"


using namespace DriveConstants;

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here
    // Configure the button bindings
    ConfigureButtonBindings();
    ConfigureBindings();

    // Set up default drive command
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    // m_drive.SetDefaultCommand(frc2::RunCommand(
    //     [this]
    //     {
    //         m_drive.Drive(
    //             -units::meters_per_second_t{frc::ApplyDeadband(
    //                 m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
    //             -units::meters_per_second_t{frc::ApplyDeadband(
    //                 m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
    //             -units::radians_per_second_t{frc::ApplyDeadband(
    //                 m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
    //             true);
    //     },
    //     {&m_drive}));
    // DoStuff = frc2::FunctionalCommand(
    //     [this] {m_elevator.setPosition(20);},
    //     [this] {},
    //     [this](bool interrupted) {},
    //     [this] {return m_elevator.getPosition() >= 19.99;},
    //     {&m_elevator}
    // )
    // DoStuff2 = frc2::FunctionalCommand(
    //     [this] {m_elevator.setPosition(5);},
    //     [this] {},
    //     [this](bool interrupted) {},
    //     [this] {return m_elevator.getPosition() <= 5.1;},
    //     {&m_elevator}
    // )
    // ElevatorSequence = frc2::SequentialCommandGroup(
    //     std::move(DoStuff), 
    //     std::move(DoStuff2)
    // ).ToPtr();
    //m_driverController.A().OnTrue(frc2::cmd::RunOnce([this] {m_elevator.setPosition(20);}).AlongWith(frc2::cmd::WaitUntil([this] {m_elevator.getPosition()>=19.99;})));

    if(m_elevator.CompBotSettings==false)
    {
        // m_driverController.A().OnTrue(frc2::cmd::RunOnce([this]
        // { m_elevator.setPosition(20); }).AlongWith(frc2::cmd::WaitUntil([this]
        // { return m_elevator.getPosition() >= 19.5; })).AndThen(frc2::cmd::RunOnce([this]
        // {m_elevator.setPosition(5);}).AlongWith(frc2::cmd::WaitUntil([this]{return m_elevator.getPosition()<=5.5;})))
        // .AndThen((DeployAlgae(&intake))));

        m_driverController.A().OnTrue(frc2::cmd::RunOnce([this]
        { m_elevator.setPosition(20); }).AlongWith(frc2::cmd::WaitUntil([this]{ return m_elevator.getPosition() >= 19.8; }))
        .AndThen(frc2::cmd::RunOnce([this]{m_elevator.setPosition(5); intake.SetAngle(165_deg); intake.Intake(0.3);})).AlongWith(frc2::cmd::WaitUntil([this]{ return m_elevator.getPosition()<=5.2;}))
        .AndThen(StopDeploy(&intake))).AlongWith(frc2::cmd::WaitUntil([this] {return m_elevator.getPosition()<=4.5;}));

    }
    else
    {                                                                                             
        m_driverController.A().OnTrue(frc2::cmd::RunOnce([this]
        { m_elevator.setPosition(15); }).AlongWith(frc2::cmd::WaitUntil([this]
        { return m_elevator.getPosition() >= 14.5; })).AndThen(frc2::cmd::RunOnce([this]
        {m_elevator.setPosition(10);})));
    }
     
} 

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::cmd::Print("No Autonomous Command!");
}

void RobotContainer::ConfigureButtonBindings()
{
    
}

void RobotContainer::ConfigureBindings()
{
    // m_driverController.A().OnTrue(frc2::cmd::RunOnce([this]
    //                                                       {m_elevator.setPosition(0.5);},{&m_elevator}));
    // m_driverController.B().OnTrue(frc2::cmd::RunOnce([this]
    //                                                     {m_elevator.setPosition(20);},{&m_elevator}));

    //drive
    m_driverController.LeftBumper().WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
    m_driverController.X().WhileTrue(new frc2::RunCommand([this] { m_drive.ZeroHeading(); }, {&m_drive})); 

    //algae intake
    // m_driverController.RightBumper().OnTrue(IntakeAlgae(&intake));
    // m_driverController.RightBumper().OnFalse(StopIntake(&intake));

    // m_driverController.RightTrigger().OnTrue(DeployAlgae(&intake));
    // m_driverController.RightTrigger().OnFalse(StopDeploy(&intake));
    
    //elevator
    // m_driverController.A().OnTrue(&DoStuff);
    // m_driverController.B().OnTrue(&DoStuff2);
    // m_driverController.A().OnTrue(std::move(ElevatorSequence));
   
}