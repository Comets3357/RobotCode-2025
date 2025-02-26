#include "wrapperclasses/GyroWrapper.h"

GyroWrapper::GyroWrapper() 
{

}

void GyroWrapper::ZeroGyro()
{
    CurrentYaw = units::degree_t{0} - (units::degree_t)m_gyro.GetYaw(); 
}

void GyroWrapper::SetAngle(units::degree_t angle)
{
    CurrentYaw = (angle - (units::degree_t)m_gyro.GetYaw()); 
}

frc::Rotation2d GyroWrapper::Get2DRotation()
{
    
    frc::Rotation2d temp{m_gyro.GetRotation2d().Degrees() + CurrentYaw};
    return temp; 
}

units::turn_t GyroWrapper::GetYaw()
{
    return (units::turn_t)((units::degree_t)m_gyro.GetYaw() + CurrentYaw); 
}

units::turns_per_second_t GyroWrapper::GetAngularVelocityYaw()
{
    return m_gyro.GetAngularVelocityYaw(); 
}