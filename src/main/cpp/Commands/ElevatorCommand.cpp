#include "Commands/ElevatorCommand.h"

// frc2::CommandPtr ExtendElevator(ElevatorSubsystem* elevator) {return frc2::cmd::Run([elevator] {elevator->ElevatorExtend();});}
// frc2::CommandPtr RetractElevator(ElevatorSubsystem* elevator) {return frc2::cmd::Run([elevator] {elevator->ElevatorRetract();});}
// frc2::CommandPtr StopElevator(ElevatorSubsystem* elevator) {return frc2::cmd::Run([elevator] {elevator->ElevatorStop();});}

elevatorCommand::elevatorCommand(ElevatorSubsystem* elevator)
{
    AddRequirements(elevator);
}

//void elevatorCommand::Execute()
//{  //    while(controller.A() == true) //    {
//     elevatorSubsystem->ElevatorExtend();
//    };
//    while (controller.A()==false)
//    {
//     elevatorSubsystem->ElevatorStop;
//    };
//    while(controller.B()==true)
//    {
//     elevatorSubsystem->ElevatorRetract();
//    };
//    while (controller.B()==false)
//    {
//     elevatorSubsystem->ElevatorStop;
//    };
//frc2::CommandPtr elevatorMax(ElevatorSubsystem* elevatorSubsystem) {return frc2::cmd::Run([elevatorSubsystem]{elevatorSubsystem->setPosition(1);});}

//frc2::CommandPtr elevatorMax(ElevatorSubsystem* elevator) {return frc2::cmd::Run([elevator] {elevator->setPosition(4);});}