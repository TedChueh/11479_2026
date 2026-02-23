// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

IntakeSubsystem::IntakeSubsystem(
    int intakeRightID, int intakeLeftID,
    int armRightID, int armLeftID,
    DualMotorModule::Config intakeConfig,
    DualMotorModule::Config armConfig
): intakeModule{intakeRightID, intakeLeftID, intakeConfig}, armModule{armRightID, armLeftID, armConfig} {}

frc2::CommandPtr IntakeSubsystem::Intaking(std::function<TPS()> intakeTps) {
  return frc2::cmd::Run(
      [this, intakeTps] {
        ActivateIntake(intakeTps());
      },{this}
  );
}

frc2::CommandPtr IntakeSubsystem::Lifting(Turn turns) {
  return frc2::cmd::RunOnce(
      [this, turns] {
        LiftByTurns(turns);
      },{this}
  );
}

frc2::CommandPtr IntakeSubsystem::Stop() {
  return frc2::cmd::Run(
      [this] {
        DeactivateIntake();
      },{this}
  );
}

void IntakeSubsystem::ActivateIntake(TPS tps) {
  intakeModule.motorLeft.SetControl(intakeModule.velocityControl.WithVelocity(tps));
  intakeModule.motorRight.SetControl(intakeModule.velocityControl.WithVelocity(tps));
}

void IntakeSubsystem::DeactivateIntake() {
  intakeModule.motorLeft.SetControl(controls::NeutralOut{});
  intakeModule.motorRight.SetControl(controls::NeutralOut{});
}

void IntakeSubsystem::LiftByTurns(Turn turns) {
  auto leftPos  = armModule.motorLeft.GetPosition().GetValue();
  auto rightPos = armModule.motorRight.GetPosition().GetValue();

  Turn leftTarget  = leftPos  - turns;
  Turn rightTarget = rightPos - turns;

  auto s1 = armModule.motorLeft.SetControl(armModule.motionMagicControl.WithPosition(leftTarget));
  auto s2 = armModule.motorRight.SetControl(armModule.motionMagicControl.WithPosition(rightTarget));
  fmt::print("pressed  s1={}  s2={}\n", s1.GetName(), s2.GetName());
}