
#pragma once

#include <rev/SparkBase.h>
#include <redux/sensors/Canandgyro.h>
#include <redux/canand/CanandEventLoop.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>

class GyroWrapper
{
    public: 
        GyroWrapper(); 
        
        void ZeroGyro(); 
        void SetAngle(units::degree_t angle);
        frc::Rotation2d Get2DRotation(); 
        units::turn_t GetYaw(); 
        units::turns_per_second_t GetAngularVelocityYaw(); 
    private: 
        redux::sensors::canandgyro::Canandgyro m_gyro{9};
        units::degree_t CurrentYaw = units::degree_t{0};
};
