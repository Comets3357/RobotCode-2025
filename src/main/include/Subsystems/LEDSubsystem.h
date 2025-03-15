#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
//#include <rev/CANSparkMax.h>
#include <frc/AddressableLED.h>


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
        void Periodic() override;
        LEDSubsystem()
        {
            LED1.SetLength(31);
            for (size_t i = 0; i < 31; i++)
            {
                LED_DATA[i].SetRGB(255,255,255);
            }
            LED1.SetData(LED_DATA);
            LED1.Start();
        }



    private:

    // m_secondaryController.POVUp().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((50));},{&m_elevator})
    //      .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() > (49.5);}))
    //      .AndThen(frc2::cmd::RunOnce([this] {m_elbowSubsystem.setElbowAngle(240);},{&m_elbowSubsystem})));


    //      m_secondaryController.LeftBumper().OnTrue(frc2::cmd::RunOnce([this]{ m_elevator.setPosition((32)); m_elbowSubsystem.setRollerSpeed(-0.1); },{&m_elbowSubsystem, &m_elevator})
    //      .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elevator.getAPosition() < (32.5);}))
    //      .AndThen(frc2::cmd::RunOnce([this]{m_elbowSubsystem.setElbowAngle(180); m_elbowSubsystem.setRollerSpeed(0); },{&m_elbowSubsystem})
    //      .AlongWith(frc2::cmd::WaitUntil( [this] { return m_elbowSubsystem.getElbowAngle()<=185;})))
    //      .AndThen(frc2::cmd::RunOnce([this]{ m_elevator.setPosition(3); },{&m_elevator}))); 

    //      m_secondaryController.LeftTrigger().OnTrue(frc2::cmd::RunOnce([this] {m_elevator.setPosition(m_elevator.getAPosition() - 0.5)};));
    
};


