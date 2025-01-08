// #include "wrapperclasses/SparkFlexMotor.h"

// #include "wrapperclasses/SparkMaxMotor.h"

// // SparkMaxMotor::SparkMaxMotor(int id) //: motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless} 
// // {
// //     //motor.Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kPersistParameters); 
// // }

// void SparkFlexMotor::SetPercent(double percent)
// {
//     motor.Set(percent); 
// }

// void SparkFlexMotor::StopMotor() 
// {
//     motor.Set(0); 
// }

// double SparkFlexMotor::GetRelativeVelocity() 
// {
//     return RelativeEncoder.GetVelocity(); 
// }

// double SparkFlexMotor::GetRelativePosition()
// {
//     return RelativeEncoder.GetPosition(); 
// }

// double SparkFlexMotor::SetRelativePosition(double pos)
// {
//     RelativeEncoder.SetPosition(pos); 
// }

// void SparkFlexMotor::setPID(double p, double i, double d, double ff)
// {
//     ConfigParam.P = p; 
//     ConfigParam.I = i; 
//     ConfigParam.D = d; 
//     ConfigParam.ff = ff; 
//     closedLoopConfig.Pidf(p, i, d, ff); // add slot 
// }

// void SparkFlexMotor::setPID(double p, double i, double d)
// {
//     ConfigParam.P = p; 
//     ConfigParam.I = i; 
//     ConfigParam.D = d; 
//     ConfigParam.ff = 0; 
//     closedLoopConfig.Pidf(p, i, d, 0); // add slot 
// }

// void SparkFlexMotor::setForwardSoftLimit(double limit)
// {
//     ConfigParam.forwardSoftLimit = limit; 
//     softLimitConfig.ForwardSoftLimit(limit); 
// }

// void SparkFlexMotor::setReverseSoftLimit(double limit)
// {
//     ConfigParam.reverseSoftLimit = limit; 
//     softLimitConfig.ReverseSoftLimit(limit); 
// }

// void SparkFlexMotor::enableForwardSoftLimit(bool enab)
// {
//     ConfigParam.isForwardSoftLimitEnabled = enab; 
//     softLimitConfig.ForwardSoftLimitEnabled(enab); 
// }

// void SparkFlexMotor::enableReverseSoftLimit(bool enab)
// {
//     ConfigParam.isReverseSoftLimitEnabled = enab; 
//     softLimitConfig.ReverseSoftLimitEnabled(enab); 
// }

// void SparkFlexMotor::setInverted(bool b)
// {
//     config.Inverted(b); 
// }

// void SparkFlexMotor::configure() 
// {
//     config.Apply(closedLoopConfig); 
//     config.Apply(softLimitConfig); 
// }

