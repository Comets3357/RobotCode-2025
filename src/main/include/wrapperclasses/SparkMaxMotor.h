# pragma once 

#include "wrapperclasses/Motor.h"

//#include <ClosedLoopConfig.h>
#include <rev/SparkMax.h>
#include <rev/SparkBase.h>
#include <rev/SparkClosedLoopController.h>
//#include <SparkBaseConfig.h> 
//template <typename t>


class SparkMaxMotor : public Motor {
    private: 
        rev::spark::SparkMax motor; 
        rev::spark::SparkAbsoluteEncoder AbsoluteEncoder = motor.GetAbsoluteEncoder(); 
        rev::spark::SparkRelativeEncoder RelativeEncoder = motor.GetEncoder(); 
        rev::spark::SparkClosedLoopController closedLoopController = motor.GetClosedLoopController(); 
        rev::spark::SparkBaseConfig config; 
        // rev::spark::ClosedLoopConfig closedLoopConfig; 
        // rev::spark::SoftLimitConfig softLimitConfig; 
        // rev::spark::LimitSwitchConfig limitSwitchConfig; 
        // rev::spark::AbsoluteEncoderConfig absoluteEncoderConfig; 
        // rev::spark::EncoderConfig encoderConfig; 

        
    
        
    
    public: 
            // CONSTURCTOR // 
        SparkMaxMotor(int id) : motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless} 
        {
            setPID(0, 0, 0); 
            config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake); // default idle mode to break
        }

        
        
        void SetPercent(double percent) override;
        void StopMotor() override; 
        void configure() override; 

        double GetRelativeVelocity() override; 
        double GetRelativePosition() override;
        void SetRelativePosition(double pos) override;
        double GetAbsolutePosition() override;
        double GetAbsoluteVelocity() override; 

        // CONFIGURE SETTINGS // 

        

        void SetSmartCurrentLimit(double lim) override; 

        void setFeedbackSensor(encoderType encoder) override; 
        void setInverted(bool b) override; 



                // CLOSED LOOP CONFIG // 
        void setPID(double p, double i, double d, double ff) override; 
        void setPID(double p, double i, double d) override; 

        void setMinOutput(double min) override;
        void setMaxOutput(double max) override;
        void setOutputRange(double min, double max) override;
        void setPositionWrappingEnabled(bool enabled) override;
        void setPositionWrapingMinInput(double minInput) override;
        void setPositionWrappingMaxInput(double maxInput) override;
        void setPositionWrappingMaxRange(double minInput, double maxInput) override; 
        void setReference(double ref, controlType ctrl) override; 



                // ABSOLUTE ENCODER CONFIG // 

        void setAbsolutePositionConversionFactor(double factor) override; 
        void zeroOffset(double offset) override; 
        void setAbsoluteVelocityConversionFactor(double factor) override; 



        void setForwardSoftLimit(double limit) override; 
        void setReverseSoftLimit(double limit) override; 
        void enableForwardSoftLimit(bool enab) override; 
        void enableReverseSoftLimit(bool enab) override; 

        
}; 
 