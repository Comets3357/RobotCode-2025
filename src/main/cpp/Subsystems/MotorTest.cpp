#include "Subsystems/MotorTest.h"

MotorTest::MotorTest() 
{
    WristMotor.configure();
}

void MotorTest::setSpeed(double speed)
{
    WristMotor.SetPercent(speed);
}
