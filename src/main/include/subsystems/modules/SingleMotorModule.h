// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"

using namespace ctre::phoenix6;
using namespace units;

class SingleMotorModule {
 public:

  struct Config {
    bool motorInvert = false;

    double kS = 0;
    double kV = 0;
    double kA = 0;
    double kP = 0;
    double kI = 0;
    double kD = 0;

    voltage::volt_t   PeakVoltage = 8_V;
    current::ampere_t PeakCurrent = 40_A;
    time::second_t    RampPeriod  = 0.1_s;
    angular_velocity::turns_per_second_t             MotionMagicCruiseVelocity = 0_tps; 
    angular_acceleration::turns_per_second_squared_t MotionMagicAcceleration   = 0_tr_per_s_sq;
    angular_jerk::turns_per_second_cubed_t           MotionMagicJerk           = 0_tr_per_s_cu;
  };

  SingleMotorModule(
    int motorID,
    Config config = {}
  );               

  static constexpr CANBus kCANBus = CANBus::RoboRIO();
  controls::VelocityVoltage    velocityControl    = controls::VelocityVoltage{0_tps}.WithSlot(0);
  controls::MotionMagicVoltage motionMagicControl = controls::MotionMagicVoltage{0_tr}.WithSlot(0);
  controls::NeutralOut brake{};

  hardware::TalonFX motor;
  configs::TalonFXConfiguration motorConfigs{};
};
