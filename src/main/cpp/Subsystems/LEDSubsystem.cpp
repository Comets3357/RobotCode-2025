#pragma once
#include "subsystems/LEDSubsystem.h"
#include <frc/DriverStation.h>
// #include "subsystems/IndexerSubsystem.h"

LEDSubsystem::LEDSubsystem(ClimbSubsystem* m_climb)
{
    m_climbPointer = m_climb; 
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

    // flashing purple = human playaer signal // 
    // climb color = blue // 
    // gyro zero = orange
    
    LED1.SetLength(31);
    LED1.Start();

    
    if(!enabled)
    {
        if(comms)
        {
            for (size_t i = 0; i <31; i++)
            {
                LED_DATA[i].SetRGB(0,255,0);
            }
            LED1.SetData(LED_DATA);
            // LED2.SetData(LED_DATA);
        }
        else
        {
            for (size_t i = 0; i <31; i++)
            {
                LED_DATA[i].SetRGB(255,0,0);
            }
            LED1.SetData(LED_DATA);
            //LED2.SetData(LED_DATA);
        }
        // else if(gyroZero)
        // {
        //     for (size_t i = 0; i <31; i++)
        //     {
        //         LED_DATA[i].SetRGB(255,128,0);
        //     }
        //     LED1.SetData(LED_DATA);
        //     //LED2.SetData(LED_DATA);
        // }
            
    }
    if(enabled)
    {        
           for (size_t i = 0; i <31; i++)
            {
                LED_DATA[i].SetRGB(255,255,255);
            }
            LED1.SetData(LED_DATA);
            //LED2.SetData(LED_DATA);

            if(climbReady)
            {
                for (size_t i = 0; i <31; i++)
                {
                    LED_DATA[i].SetRGB(0,0,255);
                }
                LED1.SetData(LED_DATA);
            //LED2.SetData(LED_DATA);
           }
           else if(climbRunning)
           {
                for (size_t i = 0; i <31; i++)
                {
                    LED_DATA[i].SetRGB(255,0,0);
                }
                LED1.SetData(LED_DATA);
           }
    }
}