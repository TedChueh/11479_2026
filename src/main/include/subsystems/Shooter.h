// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "subsystems/modules/DualMotorModule.h"
#include "subsystems/modules/SingleMotorModule.h"

using namespace ctre::phoenix6;
using TPS = units::turns_per_second_t;

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem(
    int shootRightID,
    int shootLeftID,
    int suctionID,
    int conveyerID,
    DualMotorModule::Config shootConfig,
    SingleMotorModule::Config suctionConfig,
    SingleMotorModule::Config conveyerConfig
  );

  frc2::CommandPtr Shooting(std::function<TPS()> shootTps, std::function<TPS()> suctionTps, std::function<TPS()> conveyerTps);

  frc2::CommandPtr Stop();

  void ActivateShooter(TPS tps);

  void ActivateSuction(TPS tps);
  
  void ActivateConveyer(TPS tps);
  
  void DeactivateShooter();
  
  void DeactivateSuction();
  
  void DeactivateConveyer();

//   /**
//    * Will be called periodically whenever the CommandScheduler runs.
//    */
//   void Periodic() override;

 private:
  frc::Timer m_timer; 
  DualMotorModule shootModule;
  SingleMotorModule suctionModule;
  SingleMotorModule conveyerModule;
};
