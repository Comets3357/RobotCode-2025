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

double SparkMaxMotor::GetRelativeVelocity() 
{
    RelativeEncoder.GetVelocity(); 
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
    ConfigParam.P = p; 
    ConfigParam.I = i; 
    ConfigParam.D = d; 
    ConfigParam.ff = ff; 
    closedLoopConfig.Pidf(p, i, d, ff); // add slot 
}

void SparkMaxMotor::setPID(double p, double i, double d)
{
    ConfigParam.P = p; 
    ConfigParam.I = i; 
    ConfigParam.D = d; 
    ConfigParam.ff = 0; 
    closedLoopConfig.Pidf(p, i, d, 0); // add slot 
}

void SparkMaxMotor::setForwardSoftLimit(double limit)
{
    ConfigParam.forwardSoftLimit = limit; 
    softLimitConfig.ForwardSoftLimit(limit); 
}

void SparkMaxMotor::setReverseSoftLimit(double limit)
{
    ConfigParam.reverseSoftLimit = limit; 
    softLimitConfig.ReverseSoftLimit(limit); 
}

void SparkMaxMotor::enableForwardSoftLimit(bool enab)
{
    ConfigParam.isForwardSoftLimitEnabled = enab; 
    softLimitConfig.ForwardSoftLimitEnabled(enab); 
}

void SparkMaxMotor::enableReverseSoftLimit(bool enab)
{
    ConfigParam.isReverseSoftLimitEnabled = enab; 
    softLimitConfig.ReverseSoftLimitEnabled(enab); 
}

void SparkMaxMotor::setInverted(bool b)
{
    config.Inverted(b); 
}

void SparkMaxMotor::setAbsolutePositionConversionFacotr(double factor)
{
    absoluteEncoderConfig.PositionConversionFactor(factor); 
}

void SparkMaxMotor::zeroOffset(double offset)
{
    absoluteEncoderConfig.ZeroOffset(offset); 
}

void SparkMaxMotor::setAbsoluteVelocityConversionFactor(double factor)
{
    absoluteEncoderConfig.VelocityConversionFactor(factor); 
}

void SparkMaxMotor::configure() 
{
    config.Apply(closedLoopConfig); 
    config.Apply(softLimitConfig); 
    config.Apply(absoluteEncoderConfig); 
}

