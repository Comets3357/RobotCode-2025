#include "subsystems/LEDSubsystem.h"
#include <frc/DriverStation.h>

LEDSubsystem::LEDSubsystem(DriveSubsystem* m_DriveP, ClimbSubsystem* m_climbP, ElbowSubsystem* m_elbowP) 
{
    m_climb = m_climbP;
    m_drive = m_DriveP;
    m_elbow = m_elbowP; 
    {
        LED1.SetLength(31);
        // for (size_t i = 0; i < 31; i++)
        // {
        //     LED_DATA[i].SetRGB(255, 255, 255);
        // }

       scrollRainbow.ApplyTo(LED_DATA); 
        LED1.SetData(LED_DATA);
        LED1.Start();
    }
    
}

void LEDSubsystem::Periodic()
{
    enabled = frc::DriverStation::IsEnabled();
    comms = frc::DriverStation::IsDSAttached();
    climbReady = m_climb->GetClimbAbsolutePosition() < 180 && !m_climb->isRunning();
    climbRunning = m_climb->isRunning();
    gyroZero = m_drive->gyroZero;
    gamePieceDetected = m_elbow->isGamePieceDetected();

    // flashing purple = human playaer signal //
    // climb color = blue //
    // gyro zero = orange

    LED1.SetLength(31);
    LED1.Start();

    if (!enabled)
    {
        //LED = "Not Enabled";

         if (gyroZero) // if the gyro is zeroed then it will be orange
        {
           // orange.ApplyTo(LED_DATA);
         //   LED1.SetData(LED_DATA);
            LED = "Gyro Zeroed";

            scroll.ApplyTo(LED_DATA); 
            LED1.SetData(LED_DATA); 
            // LED2.SetData(LED_DATA);
        }
        else if (comms) // if we have comms it will be green
        {
            // green.ApplyTo(LED_DATA);
            // LED1.SetData(LED_DATA);
            // // red.ApplyTo(LED_DATA);
            // LED = "Comms but disabled";
            // LED2.SetData(LED_DATA);

            LED = "Comms";

            scrollAndBreathe.ApplyTo(LED_DATA);
            LED1.SetData(LED_DATA);
        }
        else // if we don't have commms it will be red
        {
            //red.ApplyTo(LED_DATA);
            //LED1.SetData(LED_DATA);
            // LED = "No Comms";

            // scrollAndBreathe.ApplyTo(LED_DATA);
            // LED1.SetData(LED_DATA);
            // LED2.SetData(LED_DATA);
             scrollRainbow.ApplyTo(LED_DATA); 
        LED1.SetData(LED_DATA);
        }
       
    }
    else                                // if comms are enabled
    {
        if (m_drive->isAutoAligning)
        {
            double dist = m_drive->GetDistance(m_drive->AutoAlignPose);
            if (dist >= 2)
            {
                //LEDS DO NOTHING
            } else if (m_drive->inRange(m_drive->GetPose(), m_drive->AutoAlignPose)){
                blinkGreen.ApplyTo(LED_DATA);
                LED1.SetData(LED_DATA); 
            } else {
                // progress bar
                
                if (m_drive->inRange(m_drive->GetPose(), m_drive->AutoAlignPose))
                {
                    blinkGreen.ApplyTo(LED_DATA);
                    LED1.SetData(LED_DATA); 
                } else {
                frc::LEDPattern progressToTarget = green.ProgressMaskLayer([=]{ return (2 - dist) / 2;}); 
                progressToTarget.ApplyTo(LED_DATA);
                LED1.SetData(LED_DATA); 
                }
                
            }
        }
        else if (gamePieceDetected)
        {
            green.ApplyTo(LED_DATA); 
            LED1.SetData(LED_DATA);
            LED = "Game Piece";
        }
        else if (hPlayerGround) // this need to be flashing yellow
        {
            blinkPatternHPGround.ApplyTo(LED_DATA);
            LED1.SetData(LED_DATA);
            LED = "Human Player Ground";
        }
        else if (hPlayer) // this need to be flashing purple
        {
            blinkPatternHP.ApplyTo(LED_DATA);
            LED1.SetData(LED_DATA);
            LED = "Human Player";
        }
        else if (climbReady) // if climb is ready it will be green
        {
            green.ApplyTo(LED_DATA); 
            LED1.SetData(LED_DATA);
            LED = "Climb Ready";
            // LED2.SetData(LED_DATA);
        }
        else if (climbRunning) // if climb is deploying it will be red
        {
            red.ApplyTo(LED_DATA); 
            LED1.SetData(LED_DATA);
            LED = "CLimb Deploying";
        }
        else if (!gyroZero)
        {
            LED = "Enabled; Gyro not zero";
        whiteBreathe.ApplyTo(LED_DATA);
        LED1.SetData(LED_DATA);
        // LED2.SetData(LED_DATA);
        }
        else
        {
            LED = "Enabled";
        white.ApplyTo(LED_DATA);
        LED1.SetData(LED_DATA);
        }
    }

    frc::SmartDashboard::SmartDashboard::PutString("LED Status", LED);
}

/*
    when disabled: 
        gyro zero: scroll team colors (orange and gray)
        comms: green
        no comms: scroll and blink team colors
        default = rainbow I think (for fun)
    When Enabled: 
        Game Piece Detected = green
        hp floor = yellow flash
        hp station = ground flash
        climb ready = green
        climb deploying = red
        gyro not zero = blink white
        default = white

*/

// (.) (.)
