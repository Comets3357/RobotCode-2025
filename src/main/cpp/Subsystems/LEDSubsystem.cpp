#pragma once
#include "subsystems/LEDSubsystem.h"
#include <frc/DriverStation.h>
// #include "subsystems/IndexerSubsystem.h"

LEDSubsystem::LEDSubsystem(ClimbSubsystem* m_climb, DriveSubsystem* m_Drive)
{
    m_climbPointer = m_climb; 
    m_drivePointer = m_Drive; 
    {
            LED1.SetLength(31);
            for (size_t i = 0; i < 31; i++)
            {
                LED_DATA[i].SetRGB(255,255,255);
            }
            LED1.SetData(LED_DATA);
            LED1.Start();
        }
}

void LEDSubsystem::Periodic()
{
    enabled = frc::DriverStation::IsEnabled();
    comms = frc::DriverStation::IsDSAttached();
    climbReady = m_climbPointer->GetClimbAbsolutePosition() > 285 && !m_climbPointer->isRunning(); 
    climbRunning = m_climbPointer->isRunning(); 
    gyroZero = m_drivePointer->gyroZero; 

    // flashing purple = human playaer signal // 
    // climb color = blue // 
    // gyro zero = orange
    
    LED1.SetLength(31);
    LED1.Start();

    
    if(!enabled)
    {
        if(comms)                                   // if we have comms it will be green
        {
            for (size_t i = 0; i <31; i++)
            {
                LED_DATA[i].SetRGB(0,255,0);
            }
            LED1.SetData(LED_DATA);
            // LED2.SetData(LED_DATA);
        }
        else                                        // if we don't have commms it will be red
        {
            for (size_t i = 0; i <31; i++)
            {
                LED_DATA[i].SetRGB(255,0,0);
            }
            LED1.SetData(LED_DATA);
            //LED2.SetData(LED_DATA);
        }
        if(gyroZero)                                // if the gyro is zeroed then it will be orange
        {
            for (size_t i = 0; i <31; i++)
            {
                LED_DATA[i].SetRGB(255,165,0);
            }
            LED1.SetData(LED_DATA);
            //LED2.SetData(LED_DATA);
        }
            
    }
    if(enabled)
    {        
           for (size_t i = 0; i <31; i++)           // if enabled it will defualt to  white
            {
                LED_DATA[i].SetRGB(255,255,255);
            }
            LED1.SetData(LED_DATA);
            //LED2.SetData(LED_DATA);

            if(climbReady)                          // if climb is ready it will be green
            {
                for (size_t i = 0; i <31; i++)
                {
                    LED_DATA[i].SetRGB(0,0,255);
                }
                LED1.SetData(LED_DATA);
            //LED2.SetData(LED_DATA);
           }
           else if(climbRunning)                    // if climb is deploying it will be green
           {
                for (size_t i = 0; i <31; i++)
                {
                    LED_DATA[i].SetRGB(255,0,0);
                }
                LED1.SetData(LED_DATA);
           }
    }
}