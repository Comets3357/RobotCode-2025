#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
//#include <rev/CANSparkMax.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
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
        bool hPlayerGround = false; 
        bool gyroInPos = false;
        bool robotInPos = false; 

        void Periodic() override;
        LEDSubsystem(DriveSubsystem* m_DriveP, ClimbSubsystem* m_climbP);



    private:
    DriveSubsystem* m_drive;
    ClimbSubsystem* m_climb; 

    std::string LED{""}; 

    frc::Color greenColor{255, 0, 0}; 
    frc::Color redColor{0,255,0};
    frc::Color purpleColor{0,128,128};
    frc::Color orangeColor{128,128,0}; // probably not right
    frc::Color yellowColor{255,255,0};


    // different colors // 
    frc::LEDPattern purple = frc::LEDPattern::Solid(purpleColor); 
   frc::LEDPattern red = frc::LEDPattern::Solid(redColor);
    frc::LEDPattern yellow = frc::LEDPattern::Solid(yellowColor); 
    frc::LEDPattern green = frc::LEDPattern::Solid(greenColor); 
    frc::LEDPattern orange = frc::LEDPattern::Solid(orangeColor); 
    frc::LEDPattern white = frc::LEDPattern::Solid(frc::Color::kWhite); 
    
   

    frc::LEDPattern blinkPatternHP = purple.Blink(units::time::second_t{0.25}, units::time::second_t{0.25}); 
    frc::LEDPattern blinkPatternHPGround = yellow.Blink(units::time::second_t{0.25}, units::time::second_t{0.25}); 
    
};


