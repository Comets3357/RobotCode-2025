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
        closedLoopConfig.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder); 
    }
    else // sets feedback sensor to relative
    {
        closedLoopConfig.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder);
    }

    configure(configType::kClosedLoop); 
}

// For all of these closed loop functions do I need to call the configure function to apply them
// or is that done automatically? 

void SparkMaxMotor::setMinOutput(double min) 
{
    closedLoopConfig.MinOutput(min);
    configure(configType::kClosedLoop); 
}

void SparkMaxMotor::setMaxOutput(double max)
{
    closedLoopConfig.MaxOutput(max); 
    configure(configType::kClosedLoop); 
}

void SparkMaxMotor::setOutputRange(double min, double max) 
{
    closedLoopConfig.OutputRange(min, max); 
    configure(configType::kClosedLoop); 
}

void SparkMaxMotor::setPositionWrapingMinInput(double minInput) 
{
    closedLoopConfig.PositionWrappingMinInput(minInput);
    configure(configType::kClosedLoop); 
}

void SparkMaxMotor::setPositionWrappingMaxInput(double maxInput)
{
    closedLoopConfig.PositionWrappingMaxInput(maxInput);
    configure(configType::kClosedLoop); 
}

void SparkMaxMotor::setPositionWrappingMaxRange(double minInput, double maxInput)
{
    closedLoopConfig.PositionWrappingInputRange(minInput, maxInput); 
    configure(configType::kClosedLoop); 
}

void SparkMaxMotor::setPositionWrappingEnabled(bool enab)
{
    closedLoopConfig.PositionWrappingEnabled(enab); 
    configure(configType::kClosedLoop); 
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

double SparkMaxMotor::SetRelativePosition(double pos)
{
    RelativeEncoder.SetPosition(pos); 
}

void SparkMaxMotor::setPID(double p, double i, double d, double ff)
{
    // ConfigParam.P = p; 
    // ConfigParam.I = i; 
    // ConfigParam.D = d; 
    // ConfigParam.ff = ff; 
    closedLoopConfig.Pidf(p, i, d, ff); // add slot 
    
    configure(configType::kClosedLoop); 
}

void SparkMaxMotor::setPID(double p, double i, double d)
{
    // ConfigParam.P = p; 
    // ConfigParam.I = i; 
    // ConfigParam.D = d; 
    // ConfigParam.ff = 0; 
    closedLoopConfig.Pidf(p, i, d, 0); // add slot 
    configure(configType::kClosedLoop); 
}

void SparkMaxMotor::setForwardSoftLimit(double limit)
{
    //ConfigParam.forwardSoftLimit = limit; 
    softLimitConfig.ForwardSoftLimit(limit); 
    configure(configType::kSoftLimit); 
}

void SparkMaxMotor::setReverseSoftLimit(double limit)
{
    //ConfigParam.reverseSoftLimit = limit; 
    softLimitConfig.ReverseSoftLimit(limit); 
    configure(configType::kSoftLimit); 
}

void SparkMaxMotor::enableForwardSoftLimit(bool enab)
{
   // ConfigParam.isForwardSoftLimitEnabled = enab; 
    softLimitConfig.ForwardSoftLimitEnabled(enab); 
    configure(configType::kSoftLimit); 
}

void SparkMaxMotor::enableReverseSoftLimit(bool enab)
{
   // ConfigParam.isReverseSoftLimitEnabled = enab; 
    softLimitConfig.ReverseSoftLimitEnabled(enab); 
    configure(configType::kSoftLimit); 
}

void SparkMaxMotor::setInverted(bool b)
{
    config.Inverted(b); 
}

void SparkMaxMotor::setAbsolutePositionConversionFactor(double factor)
{
    absoluteEncoderConfig.PositionConversionFactor(factor); 
    configure(configType::kAbsoluteEncoder); 
}

void SparkMaxMotor::zeroOffset(double offset)
{
    absoluteEncoderConfig.ZeroOffset(offset); 
    configure(configType::kAbsoluteEncoder); 
}

void SparkMaxMotor::setAbsoluteVelocityConversionFactor(double factor)
{
    absoluteEncoderConfig.VelocityConversionFactor(factor); 
    configure(configType::kAbsoluteEncoder); 
}

void SparkMaxMotor::SetSmartCurrentLimit(double lim)
{
    config.SmartCurrentLimit(lim); 
}

void SparkMaxMotor::configure(configType c) 
{
    if (c == configType::kClosedLoop)
    {
        config.Apply(closedLoopConfig); 
    }
    else if (c == configType::kSoftLimit)
    {
        config.Apply(softLimitConfig); 
    }
    else if (c = configType::kAbsoluteEncoder)
    {
        config.Apply(absoluteEncoderConfig); 
    }
    
}

