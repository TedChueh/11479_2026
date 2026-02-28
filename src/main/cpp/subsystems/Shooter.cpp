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

CommandPtr ShooterSubsystem::Shooting(function<TPS()> shootTps) {
  return cmd::Run(
      [this, shootTps] {
          TPS currentTps = shootTps();

          if (currentTps < 20_tps) {
              systemStatus = false;
              DeactivateShooter();
              DeactivateSuction();
              DeactivateConveyer();

              SmartDashboard::PutString("Shooter Alert", "RPM too low, stopping operation");
          } 
          else {
              systemStatus = true;
              ActivateShooter(currentTps);  
              if (m_timer.HasElapsed(0.5_s)) {
                  ActivateSuction(20_tps);
                  ActivateConveyer(20_tps);
              }

              SmartDashboard::PutString("Shooter Alert", "");
          }
      },{this}
  ).BeforeStarting(
      [this] {
          m_timer.Reset();
          m_timer.Start();
      }
  );
}

CommandPtr ShooterSubsystem::Stop() {
  return cmd::Run(
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

void ShooterSubsystem::Periodic() {
   SmartDashboard::PutBoolean("Shooter Status", systemStatus);
} 
