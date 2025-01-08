// # pragma once

// #include "wrapperclasses/Motor.h"

// #include <rev/SparkBase.h>
// #include<rev/SparkFlex.h> 

// class SparkFlexMotor : public Motor
// {
//     private: 
//         rev::spark::SparkFlex motor; 
//         rev::spark::SparkAbsoluteEncoder AbsoluteEncoder = motor.GetAbsoluteEncoder(); 
//         rev::spark::SparkRelativeEncoder RelativeEncoder = motor.GetEncoder(); 
//         rev::spark::SparkFlexExternalEncoder ExternalRelativeEncoder = motor.GetExternalEncoder(); 
//         rev::spark::SparkBaseConfig config; 
//         rev::spark::ClosedLoopConfig closedLoopConfig; 
//         rev::spark::SoftLimitConfig softLimitConfig; 

//         protected:
//             void configure(configType c) override;

//     public: 
//         SparkFlexMotor(int id) : motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless} 
//         {
//             setPID(0, 0, 0); 
//         }

        

//         void SetPercent(double percent) override;
//         void StopMotor() override; 

//         double GetRelativeVelocity() override; 
//         double GetRelativePosition() override;
//         double SetRelativePosition(double pos) override;

//         // CONFIGURE SETTINGS // 

        
//         void setPID(double p, double i, double d, double ff) override; 
//         void setPID(double p, double i, double d) override; 

//         void setForwardSoftLimit(double limit) override; 
//         void setReverseSoftLimit(double limit) override; 
//         void enableForwardSoftLimit(bool enab) override; 
//         void enableReverseSoftLimit(bool enab) override; 

//         virtual void setInverted(bool b) override; 


// }; 