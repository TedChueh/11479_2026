// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

using namespace units::angular_velocity;

ShooterSubsystem::ShooterSubsystem(
  int shootRightID,   int shootLeftID, 
  int suctionID,
  int conveyerID,
  DualMotorModule::Config shootConfig,
  SingleMotorModule::Config suctionConfig,
  SingleMotorModule::Config conveyerConfig
): shootModule{shootRightID, shootLeftID, shootConfig}, suctionModule{suctionID, suctionConfig}, conveyerModule{conveyerID, conveyerConfig} {}

frc2::CommandPtr ShooterSubsystem::Shooting(turns_per_second_t shootTps, turns_per_second_t suctionTps, turns_per_second_t conveyerTps) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, shootTps] { ActivateShooter(shootTps); }, {this}),
      frc2::cmd::Wait(0.5_s),
      frc2::cmd::RunOnce([this, suctionTps] { ActivateSuction(suctionTps); }, {this}),
      frc2::cmd::RunOnce([this, conveyerTps] { ActivateConveyer(conveyerTps); }, {this})
  );
}

frc2::CommandPtr ShooterSubsystem::Stop() {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this] { DeactivateConveyer(); }, {this}),
      frc2::cmd::RunOnce([this] { DeactivateSuction(); }, {this}),
      frc2::cmd::Wait(0.5_s),
      frc2::cmd::RunOnce([this] { DeactivateShooter(); }, {this})
  );
}


void ShooterSubsystem::ActivateShooter(turns_per_second_t tps) {
  shootModule.motorLeft.SetControl(shootModule.velocityControl.WithVelocity(tps));
  shootModule.motorRight.SetControl(shootModule.velocityControl.WithVelocity(tps));
}

void ShooterSubsystem::ActivateSuction(turns_per_second_t tps) {
  suctionModule.motor.SetControl(suctionModule.velocityControl.WithVelocity(tps));
}

void ShooterSubsystem::ActivateConveyer(turns_per_second_t tps) {
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
