#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
//#include <rev/CANSparkMax.h>
#include <frc/AddressableLED.h>

#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"


class LEDSubsystem : public frc2::SubsystemBase
{
    public:
                  

        std::array<frc::AddressableLED::LEDData, 31> LED_DATA;
        frc::AddressableLED LED1{0};
        // frc::AddressableLED LED2{1};
        frc::AddressableLED::LEDData LEDColors{0,0,0};

        


        
      
        bool comms = false; 
        bool detect =false;
        bool enabled = false;  
        bool gyroZero = false; 
        bool climbReady = false; 
        bool climbRunning = false; 
        bool hPlayer = false; 


        void Periodic() override;
        LEDSubsystem(ClimbSubsystem* m_climb, DriveSubsystem* m_Drive);


    private:
        ClimbSubsystem* m_climbPointer; 
        DriveSubsystem* m_drivePointer; 
    
};


