#include "wrapperclasses/SparkFlexMotor.h"


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

void SparkFlexMotor::SetRelativePosition(double pos)
{
    RelativeEncoder.SetPosition(pos); 
}

void SparkFlexMotor::setPID(double p, double i, double d, double ff)
{
    config.closedLoop.Pidf(p, i, d, ff); 
}

void SparkFlexMotor::setPID(double p, double i, double d)
{
    config.closedLoop.Pidf(p, i, d, 0); 
}

void SparkFlexMotor::setPID(double p, double i, double d, double ff, int slot)
{
    switch(slot)
    {
        case 1:
            config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot1);
            break;
        case 2:
            config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot2);
            break;
        case 3:
            config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot3);
            break; 
         default:
            config.closedLoop.Pidf(p, i, d, ff, rev::spark::kSlot0);
    }
    
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

void SparkFlexMotor::setMinOutput(double min) 
{
    config.closedLoop.MinOutput(min);
}

void SparkFlexMotor::setMaxOutput(double max)
{
    config.closedLoop.MaxOutput(max);  
}

void SparkFlexMotor::setOutputRange(double min, double max) 
{
    config.closedLoop.OutputRange(min, max); 
}

void SparkFlexMotor::setPositionWrapingMinInput(double minInput) 
{
    config.closedLoop.PositionWrappingMinInput(minInput);
}

void SparkFlexMotor::setPositionWrappingMaxInput(double maxInput)
{
    config.closedLoop.PositionWrappingMaxInput(maxInput);   
}

void SparkFlexMotor::setPositionWrappingMaxRange(double minInput, double maxInput)
{
    config.closedLoop.PositionWrappingInputRange(minInput, maxInput);  
}

void SparkFlexMotor::setPositionWrappingEnabled(bool enab)
{
    config.closedLoop.PositionWrappingEnabled(enab); 
}

void SparkFlexMotor::setReference(double ref, controlType ctrl)
{
    if (ctrl == Motor::controlType::position)
    {
        closedLoopController.SetReference(ref, rev::spark::SparkLowLevel::ControlType::kPosition); 
    }
    else if (ctrl == Motor::controlType::velocity)
    {
        closedLoopController.SetReference(ref, rev::spark::SparkLowLevel::ControlType::kVelocity); 
    }
}

void SparkFlexMotor::SetSmartCurrentLimit(double lim)
{
    config.SmartCurrentLimit(lim); 
}