#include "subsystems/LEDSubsystem.h"
#include <frc/DriverStation.h>
// #include "subsystems/IndexerSubsystem.h"

void LEDSubsystem::Periodic()
{
    LEDSubsystem::enabled = frc::DriverStation::IsEnabled();
    LEDSubsystem::comms = frc::DriverStation::IsDSAttached();
    LED1.SetLength(31);
    LED1.Start();

    
    if(!enabled)
    {
        if(comms==true)
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
    }
}