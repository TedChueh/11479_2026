// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "subsystems/modules/DualMotorModule.h"
#include "subsystems/modules/SingleMotorModule.h"

using namespace ctre::phoenix6;
using namespace units::angular_velocity;

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

  frc2::CommandPtr Shooting(turns_per_second_t shootTps, turns_per_second_t suctionTps, turns_per_second_t conveyerTps);

  frc2::CommandPtr Stop();

  void ActivateShooter(turns_per_second_t tps);

  void ActivateSuction(turns_per_second_t tps);
  
  void ActivateConveyer(turns_per_second_t tps);

  void DeactivateShooter();
  
  void DeactivateSuction();
  
  void DeactivateConveyer();

//   /**
//    * Will be called periodically whenever the CommandScheduler runs.
//    */
//   void Periodic() override;

 private:
  DualMotorModule shootModule;
  SingleMotorModule suctionModule;
  SingleMotorModule conveyerModule;
};
