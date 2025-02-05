#include "Commands/ClimbCommand.h"

frc2::CommandPtr ClimbRetract(ClimbSubsystem* climb) {return frc2::cmd::Run([climb] {climb->ClimbRetract();});}

frc2::CommandPtr ClimbStop(ClimbSubsystem* climb) {return frc2::cmd::Run([climb] {climb->ClimbStop();});}

frc2::CommandPtr ClimbSetPositionLow(ClimbSubsystem* climb) {return frc2::cmd::Run([climb] {climb->ClimbSetPosition(90);});}

frc2::CommandPtr ClimbSetPositionHigh(ClimbSubsystem* climb) {return frc2::cmd::Run([climb] {climb->ClimbSetPosition(180);});}



// frc2::CommandPtr ClimbSetPercent(ClimbSubsystem* climb) {return frc2::cmd::Run([climb] {climb->ClimbSetPercent();});}



