#pragma once 

//template <typename t>

class Motor {
    private: 

    public:
        Motor() {}; 
       

        virtual void SetPercent(double percent) = 0;
        virtual void StopMotor() = 0; 

        // ENCODER FUNCTIONS // 

        virtual double GetRelativeVelocity() = 0; 
        virtual double GetRelativePosition() = 0; 
        virtual double GetAbsoluteVelocity() = 0; 
        virtual double GetAbsolutePosition() = 0; 

        virtual double SetRelativePosition(double pos) = 0; 

        // END ENCODER FUNCTIONS // 

        // CONFIGURE SETTINGS // 

        virtual void configure() = 0; 
        virtual void setPID(double p, double i, double d, double ff) = 0; 
        virtual void setPID(double p, double i, double d) = 0; 
        //virtual void setFeedbackSensor(t sensor) = 0; 

        virtual void setForwardSoftLimit(double limit) = 0; 
        virtual void setReverseSoftLimit(double limit) = 0; 
        virtual void enableForwardSoftLimit(bool enab) = 0; 
        virtual void enableReverseSoftLimit(bool enab) = 0; 

        virtual void setAbsolutePositionConversionFactor(double factor) = 0; 
        virtual void zeroOffset(double offset) = 0; 
        virtual void setAbsoluteVelocityConversionFactor(double factor) = 0; 

        virtual void setInverted(bool b) = 0; 


        struct ConfigParamDataType {
            double P; 
            double I; 
            double D; 
            double ff; 
            double forwardSoftLimit; 
            double reverseSoftLimit; 
            bool isForwardSoftLimitEnabled; 
            bool isReverseSoftLimitEnabled; 
            bool isInverted; 
        }; 

        ConfigParamDataType ConfigParam; 

}; 