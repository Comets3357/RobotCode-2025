#include "wrapperclasses/SparkMaxMotor.h"

// SparkMaxMotor::SparkMaxMotor(int id) //: motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless} 
// {
//     //motor.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters); 
// }

void SparkMaxMotor::SetPercent(double percent)
{
    motor.Set(percent); 
}

void SparkMaxMotor::StopMotor() 
{
    motor.Set(0); 
}

void SparkMaxMotor::setFeedbackSensor(encoderType encoder)
{
    if (encoder == Motor::encoderType::absolute) // sets feedback sensor to absolute
    {
        config.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder); 
    }
    else // sets feedback sensor to relative
    {
        config.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder);
    }


}

void SparkMaxMotor::setMinOutput(double min) 
{
    config.closedLoop.MinOutput(min);
    
}

void SparkMaxMotor::setMaxOutput(double max)
{
    config.closedLoop.MaxOutput(max); 
    
}

void SparkMaxMotor::setOutputRange(double min, double max) 
{
    config.closedLoop.OutputRange(min, max); 
    
}

void SparkMaxMotor::setPositionWrapingMinInput(double minInput) 
{
    config.closedLoop.PositionWrappingMinInput(minInput);
    
}

void SparkMaxMotor::setPositionWrappingMaxInput(double maxInput)
{
    config.closedLoop.PositionWrappingMaxInput(maxInput);
    
}

void SparkMaxMotor::setPositionWrappingMaxRange(double minInput, double maxInput)
{
    config.closedLoop.PositionWrappingInputRange(minInput, maxInput); 
    
}

void SparkMaxMotor::setPositionWrappingEnabled(bool enab)
{
    config.closedLoop.PositionWrappingEnabled(enab); 
    
}

void SparkMaxMotor::setReference(double ref, controlType ctrl)
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



double SparkMaxMotor::GetRelativeVelocity() 
{
    return RelativeEncoder.GetVelocity(); 
}

double SparkMaxMotor::GetRelativePosition()
{
    return RelativeEncoder.GetPosition(); 
}

double SparkMaxMotor::GetAbsolutePosition()
{
    return AbsoluteEncoder.GetPosition(); 
}

double SparkMaxMotor::GetAbsoluteVelocity()
{
    return AbsoluteEncoder.GetVelocity(); 
}

void SparkMaxMotor::SetRelativePosition(double pos)
{
    RelativeEncoder.SetPosition(pos); 
}

void SparkMaxMotor::setPID(double p, double i, double d, double ff)
{
    config.closedLoop.Pidf(p, i, d, ff); // add slot    
}

void SparkMaxMotor::setPID(double p, double i, double d)
{
    config.closedLoop.Pidf(p, i, d, 0); // add slot    
}

void SparkMaxMotor::setForwardSoftLimit(double limit)
{
    config.softLimit.ForwardSoftLimit(limit);   
}

void SparkMaxMotor::setReverseSoftLimit(double limit)
{
    config.softLimit.ReverseSoftLimit(limit);    
}

void SparkMaxMotor::enableForwardSoftLimit(bool enab)
{ 
    config.softLimit.ForwardSoftLimitEnabled(enab);    
}

void SparkMaxMotor::enableReverseSoftLimit(bool enab)
{
    config.softLimit.ReverseSoftLimitEnabled(enab);    
}

void SparkMaxMotor::setInverted(bool b)
{
    config.Inverted(b); 
}

void SparkMaxMotor::setAbsolutePositionConversionFactor(double factor)
{
    config.absoluteEncoder.PositionConversionFactor(factor); 
}

void SparkMaxMotor::zeroOffset(double offset)
{
    config.absoluteEncoder.ZeroOffset(offset); 
}

void SparkMaxMotor::setAbsoluteVelocityConversionFactor(double factor)
{
    config.absoluteEncoder.VelocityConversionFactor(factor); 
}

void SparkMaxMotor::SetSmartCurrentLimit(double lim)
{
    config.SmartCurrentLimit(lim); 
}

void SparkMaxMotor::configure() 
{
    motor.Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters); 
}

