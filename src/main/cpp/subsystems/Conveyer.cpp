// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Conveyer.h"

ConveyerSubsystem::ConveyerSubsystem(
  int conveyerRightID,
  int conveyerLeftID,
  DualMotorModule::Config conveyerConfig
): conveyerModule{conveyerRightID, conveyerLeftID, conveyerConfig} {}

CommandPtr ConveyerSubsystem::Conveying() {
  return cmd::Run(
      [this]{
        systemStatus = true;
        ActivateConveyer(20_tps);
      },{this}
  );
}

CommandPtr ConveyerSubsystem::Stop() {
  return cmd::Run(
      [this]{
        systemStatus = false;
        DeactivateConveyer();
      },{this}
  );
}

void ConveyerSubsystem::ActivateConveyer(TPS tps) {
  conveyerModule.motorLeft.SetControl(conveyerModule.velocityControl.WithVelocity(tps));
  conveyerModule.motorRight.SetControl(conveyerModule.velocityControl.WithVelocity(tps));
}

void ConveyerSubsystem::DeactivateConveyer() {
  conveyerModule.motorLeft.SetControl(controls::NeutralOut{});
  conveyerModule.motorRight.SetControl(controls::NeutralOut{});
}

void ConveyerSubsystem::Periodic() {
   SmartDashboard::PutBoolean("Conveyer Status", systemStatus);
} 
