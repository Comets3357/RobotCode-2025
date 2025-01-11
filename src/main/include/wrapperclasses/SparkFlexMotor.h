# pragma once

#include "wrapperclasses/Motor.h"

#include <rev/SparkBase.h>
#include<rev/SparkFlex.h> 

class SparkFlexMotor : public Motor
{
    private: 
        rev::spark::SparkFlex motor; 
        rev::spark::SparkAbsoluteEncoder AbsoluteEncoder = motor.GetAbsoluteEncoder(); 
        rev::spark::SparkRelativeEncoder RelativeEncoder = motor.GetEncoder(); 
        rev::spark::SparkFlexExternalEncoder ExternalRelativeEncoder = motor.GetExternalEncoder(); 
        rev::spark::SparkBaseConfig config; 
        rev::spark::SparkClosedLoopController closedLoopController = motor.GetClosedLoopController();

            

    public: 

        // CONSTURCTOR // 
        SparkFlexMotor(int id) : motor{id, rev::spark::SparkLowLevel::MotorType::kBrushless} 
        {
            setPID(0, 0, 0); 
        }

        void configure() override;
        

        void SetPercent(double percent) override;
        void StopMotor() override; 

        double GetRelativeVelocity() override; 
        double GetRelativePosition() override;
        void SetRelativePosition(double pos) override;

        // CONFIGURE SETTINGS // 
        
            void SetSmartCurrentLimit(double lim) override; 
            void setMinOutput(double min) override;
            void setMaxOutput(double max) override;
            void setOutputRange(double min, double max) override;
            void setPositionWrappingEnabled(bool enabled) override;
            void setPositionWrapingMinInput(double minInput) override;
            void setPositionWrappingMaxInput(double maxInput) override;
            void setPositionWrappingMaxRange(double minInput, double maxInput) override; 
            void setReference(double ref, controlType ctrl) override; 
        

        
        void setPID(double p, double i, double d, double ff) override; 
        void setPID(double p, double i, double d) override; 

        void setForwardSoftLimit(double limit) override; 
        void setReverseSoftLimit(double limit) override; 
        void enableForwardSoftLimit(bool enab) override; 
        void enableReverseSoftLimit(bool enab) override; 

        virtual void setInverted(bool b) override; 


}; 