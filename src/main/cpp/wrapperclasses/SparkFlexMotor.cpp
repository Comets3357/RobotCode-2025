#include "wrapperclasses/SparkFlexMotor.h"

#include "wrapperclasses/SparkMaxMotor.h"

// // SparkMaxMotor::SparkMaxMotor(int id) //: motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless} 
// // {
// //     //motor.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters); 
// // }

void SparkFlexMotor::SetPercent(double percent)
{
    motor.Set(percent); 
}

void SparkFlexMotor::StopMotor() 
{
    motor.Set(0); 
}

double SparkFlexMotor::GetRelativeVelocity() 
{
    return RelativeEncoder.GetVelocity(); 
}

double SparkFlexMotor::GetRelativePosition()
{
    return RelativeEncoder.GetPosition(); 
}

double SparkFlexMotor::SetRelativePosition(double pos)
{
    RelativeEncoder.SetPosition(pos); 
}

void SparkFlexMotor::setPID(double p, double i, double d, double ff)
{
    
    config.closedLoop.Pidf(p, i, d, ff); // add slot 
}

void SparkFlexMotor::setPID(double p, double i, double d)
{
    
    config.closedLoop.Pidf(p, i, d, 0); // add slot 
}

void SparkFlexMotor::setForwardSoftLimit(double limit)
{
    
    config.softLimit.ForwardSoftLimit(limit); 
}

void SparkFlexMotor::setReverseSoftLimit(double limit)
{
    
    config.softLimit.ReverseSoftLimit(limit); 
}

void SparkFlexMotor::enableForwardSoftLimit(bool enab)
{
    
    config.softLimit.ForwardSoftLimitEnabled(enab); 
}

void SparkFlexMotor::enableReverseSoftLimit(bool enab)
{
    
    config.softLimit.ReverseSoftLimitEnabled(enab); 
}

void SparkFlexMotor::setInverted(bool b)
{
    config.Inverted(b); 
}

void SparkFlexMotor::configure() 
{
    motor.Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters); 
}

// void SparkMaxMotor::setMinOutput(double min) 
// {
//     config.closedLoop.MinOutput(min);
    
// }

// void SparkMaxMotor::setMaxOutput(double max)
// {
//     config.closedLoop.MaxOutput(max); 
    
// }

// void SparkMaxMotor::setOutputRange(double min, double max) 
// {
//     config.closedLoop.OutputRange(min, max); 
    
// }

// void SparkMaxMotor::setPositionWrapingMinInput(double minInput) 
// {
//     config.closedLoop.PositionWrappingMinInput(minInput);
    
// }

// void SparkMaxMotor::setPositionWrappingMaxInput(double maxInput)
// {
//     config.closedLoop.PositionWrappingMaxInput(maxInput);
    
// }

// void SparkMaxMotor::setPositionWrappingMaxRange(double minInput, double maxInput)
// {
//     config.closedLoop.PositionWrappingInputRange(minInput, maxInput); 
    
// }

// void SparkMaxMotor::setPositionWrappingEnabled(bool enab)
// {
//     config.closedLoop.PositionWrappingEnabled(enab); 
    
// }

// void SparkMaxMotor::setReference(double ref, controlType ctrl)
// {
//     if (ctrl == Motor::controlType::position)
//     {
//         closedLoopController.SetReference(ref, rev::spark::SparkLowLevel::ControlType::kPosition); 
//     }
//     else if (ctrl == Motor::controlType::velocity)
//     {
//         closedLoopController.SetReference(ref, rev::spark::SparkLowLevel::ControlType::kVelocity); 
//     }
   
// }