#pragma once

// template <typename t>

class Motor
{
private:
public:
    Motor() {};

    enum encoderType
    {
        absolute,
        relative
    };

    enum controlType
    {
        position,
        velocity
    };

    virtual void SetPercent(double percent) = 0;
    virtual void StopMotor() = 0;
    virtual void configure() = 0;

    // ENCODER FUNCTIONS //

    virtual double GetRelativeVelocity() = 0;
    virtual double GetRelativePosition() = 0;
    virtual double GetAbsoluteVelocity() = 0;
    virtual double GetAbsolutePosition() = 0;

    virtual void SetRelativePosition(double pos) = 0;

    // END ENCODER FUNCTIONS //

    // CONFIGURE SETTINGS //

    virtual void SetSmartCurrentLimit(double lim) = 0;
    virtual void setFeedbackSensor(encoderType encoder) = 0;

    virtual void setRelativeVelocityConversionFactor(double fac) = 0;
    virtual void setRelativePositionConversionFactor(double fac) = 0;

    // all closed loop configure settings //
    virtual void setPID(double p, double i, double d, double ff) = 0;
    virtual void setPID(double p, double i, double d, double ff, int slot) = 0;
    virtual void setPID(double p, double i, double d) = 0;
    virtual void setMinOutput(double min) = 0;
    virtual void setMaxOutput(double max) = 0;
    virtual void setOutputRange(double min, double max) = 0;
    virtual void setPositionWrappingEnabled(bool enabled) = 0;
    virtual void setPositionWrapingMinInput(double minInput) = 0;
    virtual void setPositionWrappingMaxInput(double maxInput) = 0;
    virtual void setPositionWrappingMaxRange(double minInput, double maxInput) = 0;
    virtual void setReference(double ref, controlType ctrl) = 0;

    virtual void setForwardSoftLimit(double limit) = 0;
    virtual void setReverseSoftLimit(double limit) = 0;
    virtual void enableForwardSoftLimit(bool enab) = 0;
    virtual void enableReverseSoftLimit(bool enab) = 0;

    virtual void setAbsolutePositionConversionFactor(double factor) = 0;
    virtual void zeroOffset(double offset) = 0;
    virtual void setAbsoluteVelocityConversionFactor(double factor) = 0;

    virtual void setInverted(bool b) = 0;
};