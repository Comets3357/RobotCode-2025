
#pragma once
#include "subsystems/LEDSubsystem.h"
#include <frc/DriverStation.h>
// #include "subsystems/IndexerSubsystem.h"

LEDSubsystem::LEDSubsystem(DriveSubsystem* m_DriveP, ClimbSubsystem* m_climbP)
{
    m_climb = m_climbP; 
    m_drive = m_DriveP; 
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
    climbReady = m_climb->GetClimbAbsolutePosition() > 285 && !m_climb->isRunning(); 
    climbRunning = m_climb->isRunning(); 
    gyroZero = m_drive->gyroZero; 

    // flashing purple = human playaer signal // 
    // climb color = blue // 
    // gyro zero = orange
    
    LED1.SetLength(31);
    LED1.Start();

    
    if(!enabled)
    {
        LED = "Not Enabled"; 
        if(comms)                                   // if we have comms it will be green
        {
            for (size_t i = 0; i <31; i++)
            {
                LED_DATA[i].SetRGB(0,255,0);
            }
            LED1.SetData(LED_DATA);
            LED = "Comms";
            // LED2.SetData(LED_DATA);
        }
        else                                        // if we don't have commms it will be red
        {
            for (size_t i = 0; i <31; i++)
            {
                LED_DATA[i].SetRGB(255,0,0);
            }
            LED1.SetData(LED_DATA);
            LED = "No Comms";
            //LED2.SetData(LED_DATA);
        }
        if(gyroZero)                                // if the gyro is zeroed then it will be orange
        {
            for (size_t i = 0; i <31; i++)
            {
                LED_DATA[i].SetRGB(255,165,0);
            }
            LED1.SetData(LED_DATA);
            LED = "Gyro Zeroed";
            //LED2.SetData(LED_DATA);
        }
            
    }
    if(enabled)
    {        
        LED = "Enabled"; 
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
                LED = "Climb Ready";
            //LED2.SetData(LED_DATA);
           }
           else if(climbRunning)                    // if climb is deploying it will be red
           {
                for (size_t i = 0; i <31; i++)
                {
                    LED_DATA[i].SetRGB(255,0,0);
                }
                LED1.SetData(LED_DATA);
                LED = "CLimb Deploying";
           }

           if (hPlayer)                             // this need to be flashing purple
           {
            for (size_t i = 0; i <31; i++)
                {
                    LED_DATA[i].SetRGB(255,0,0);
                }
                LED1.SetData(LED_DATA);
                LED = "Human Player";
           }

           if (hPlayerGround)                             // this need to be flashing yellow
           {
            for (size_t i = 0; i <31; i++)
                {
                    LED_DATA[i].SetRGB(255,0,0);
                }
                LED1.SetData(LED_DATA);
                LED = "Human Player Ground"; 
           }
    }

    frc::SmartDashboard::SmartDashboard::PutString("LED Status", LED); 
}
