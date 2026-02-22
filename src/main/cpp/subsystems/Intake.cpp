// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

IntakeSubsystem::IntakeSubsystem(
    int intakeRightID, int intakeLeftID,
    int intakeRightArmID, int intakeLeftArmID,
    DualMotorModule::Config intakeConfig,
    DualMotorModule::Config liftConfig
): intakeModule{intakeRightID, intakeLeftID, intakeConfig}, liftModule{intakeRightArmID, intakeLeftArmID, liftConfig} {}

frc2::CommandPtr IntakeSubsystem::Intaking(std::function<TPS()> intakeTps) {
  return frc2::cmd::Run(
      [this, intakeTps] {
        ActivateIntake(intakeTps());
      },{this}
    );
}

frc2::CommandPtr IntakeSubsystem::Stop() {
  return frc2::cmd::Run([this] {
    DeactivateIntake();
  }, {this});
}

frc2::CommandPtr IntakeSubsystem::Lifting(Turn Turns) {
  return frc2::cmd::RunOnce([this, Turns] {
      LiftByTurns(Turns);
    }, {this});
}

void IntakeSubsystem::ActivateIntake(TPS tps) {
  intakeModule.motorLeft.SetControl(intakeModule.velocityControl.WithVelocity(tps));
  intakeModule.motorRight.SetControl(intakeModule.velocityControl.WithVelocity(tps));
}

void IntakeSubsystem::DeactivateIntake() {
  intakeModule.motorLeft.SetControl(controls::NeutralOut{});
  intakeModule.motorRight.SetControl(controls::NeutralOut{});
}

void IntakeSubsystem::LiftByTurns(Turn Turns) {
  auto leftPos  = liftModule.motorLeft.GetPosition().GetValue();
  auto rightPos = liftModule.motorRight.GetPosition().GetValue();

  Turn leftTarget  = leftPos  + Turns;
  Turn rightTarget = rightPos + Turns;

  liftModule.motorLeft.SetControl(liftModule.motionMagicControl.WithPosition(leftTarget));
  liftModule.motorRight.SetControl(liftModule.motionMagicControl.WithPosition(rightTarget));
}