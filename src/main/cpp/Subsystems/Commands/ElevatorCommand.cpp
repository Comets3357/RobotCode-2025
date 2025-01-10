#include "Commands/ElevatorCommand.h"

frc2::CommandPtr ExtendElevator(ElevatorSubsystem* elevator) {return frc2::cmd::Run([elevator] {elevator->ElevatorExtend();});}

frc2::CommandPtr RetractElevator(ElevatorSubsystem* elevator) {return frc2::cmd::Run([elevator] {elevator->ElevatorRetract();});}

frc2::CommandPtr StopElevator(ElevatorSubsystem* elevator) {return frc2::cmd::Run([elevator] {elevator->ElevatorStop();});}