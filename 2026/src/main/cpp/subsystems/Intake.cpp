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

CommandPtr IntakeSubsystem::Intaking(function<TPS()> intakeTps) {
  return cmd::Run(
      [this, intakeTps] {
        intakeStatus = true;
        ActivateIntake(intakeTps());
      },{this}
  );
}

CommandPtr IntakeSubsystem::StopIntaking() {
  return cmd::Run(
      [this] {
        intakeStatus = false;
        DeactivateIntake();
      },{this}
  );
}

CommandPtr IntakeSubsystem::ManualArmControl(function<double()> joystickValue) {
  return cmd::Run(
      [this, joystickValue] {
        LiftByOpenLoop(joystickValue());
      },{this}
  );
}

CommandPtr IntakeSubsystem::Lifting() {
  return cmd::RunOnce(
      [this] {
        if(!armStatus && !isArmActive()) {
          armStatus = true;
          LiftByTurns(113_tr);
        }
      },{this}
  );
}

CommandPtr IntakeSubsystem::Lowering() {
  return cmd::RunOnce(
      [this] {
        if(armStatus && !isArmActive()) {
          armStatus = false;
          LiftByTurns(-113_tr);
        }
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

  leftTarget  = leftPos  - turns;
  rightTarget = rightPos - turns;

  armModule.motorLeft.SetControl(armModule.motionMagicControl.WithPosition(leftTarget));
  armModule.motorRight.SetControl(armModule.motionMagicControl.WithPosition(rightTarget));
}

void IntakeSubsystem::LiftByOpenLoop(double dutyPercentage) {
  armModule.motorLeft.SetControl(armModule.dutyCycleControl.WithOutput(dutyPercentage));
  armModule.motorRight.SetControl(armModule.dutyCycleControl.WithOutput(dutyPercentage));
}

bool IntakeSubsystem::isIntakeActive() const {
  return intakeStatus;
}

bool IntakeSubsystem::isArmActive() {
  auto errorLeft = armModule.motorLeft.GetPosition().GetValueAsDouble() - leftTarget.value();
  auto errorRight = armModule.motorRight.GetPosition().GetValueAsDouble() - rightTarget.value();
  return (abs(errorLeft) < kToleranceRot.value() && abs(errorRight) < kToleranceRot.value());
}

void IntakeSubsystem::Periodic() {
    SmartDashboard::PutBoolean("Intake Status", intakeStatus);
    SmartDashboard::PutString("Arm Status", armStatus ? "Lifting↑" : "Lowering↓"  );
}