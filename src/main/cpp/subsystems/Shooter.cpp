// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

ShooterSubsystem::ShooterSubsystem(
  int shootRightID,   int shootLeftID, 
  int suctionID,
  int conveyerID,
  DualMotorModule::Config shootConfig,
  SingleMotorModule::Config suctionConfig,
  SingleMotorModule::Config conveyerConfig
): shootModule{shootRightID, shootLeftID, shootConfig}, suctionModule{suctionID, suctionConfig}, conveyerModule{conveyerID, conveyerConfig} {}

frc2::CommandPtr ShooterSubsystem::Shooting(std::function<TPS()> shootTps) {
  return frc2::cmd::Run(
      [this, shootTps] {
        ActivateShooter(shootTps());
        std::function<TPS()> deltaTps = [shootTps] {
          if(shootTps() > 20_tps) {
            return 20_tps;
          } else {
            return 0_tps;
          }
        };
        if(m_timer.HasElapsed(0.5_s)) {
          ActivateSuction(deltaTps());
          ActivateConveyer(deltaTps());
        }
      },{this}
    ).BeforeStarting(
      [this] {
        m_timer.Reset();
        m_timer.Start();
      }
  );
}

frc2::CommandPtr ShooterSubsystem::Stop() {
  return frc2::cmd::Run(
      [this]{
        DeactivateShooter();
        DeactivateSuction();
        DeactivateConveyer();
      },{this}
  );
}

void ShooterSubsystem::ActivateShooter(TPS tps) {
  shootModule.motorLeft.SetControl(shootModule.velocityControl.WithVelocity(tps));
  shootModule.motorRight.SetControl(shootModule.velocityControl.WithVelocity(tps));
}

void ShooterSubsystem::ActivateSuction(TPS tps) {
  suctionModule.motor.SetControl(suctionModule.velocityControl.WithVelocity(tps));
}

void ShooterSubsystem::ActivateConveyer(TPS tps) {
  conveyerModule.motor.SetControl(conveyerModule.velocityControl.WithVelocity(tps));
}

void ShooterSubsystem::DeactivateShooter() {
  shootModule.motorLeft.SetControl(controls::NeutralOut{});
  shootModule.motorRight.SetControl(controls::NeutralOut{});
}

void ShooterSubsystem::DeactivateSuction() {
  suctionModule.motor.SetControl(controls::NeutralOut{});
}

void ShooterSubsystem::DeactivateConveyer() {
  conveyerModule.motor.SetControl(controls::NeutralOut{});
}

// void Shooter::Periodic() {
//   // Implementation of subsystem periodic method goes here.
// }
