// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"

#include <frc/Timer.h>
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "subsystems/modules/DualMotorModule.h"
#include "subsystems/modules/SingleMotorModule.h"

using namespace ctre::phoenix6;
using TPS = units::turns_per_second_t;
using Turn = units::turn_t;

class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem(
            int intakeRightID, 
            int intakeLeftID, 
            int armRightID, 
            int armLeftID,
            DualMotorModule::Config intakeConfig,
            DualMotorModule::Config armConfig
        );

        frc2::CommandPtr Intaking(std::function<TPS()> intakeTps);
        frc2::CommandPtr Lifting(Turn turns);
        frc2::CommandPtr Stop();

        void ActivateIntake(TPS tps);
        void DeactivateIntake();
        void LiftByTurns(Turn turns);

    private:
        DualMotorModule intakeModule;
        DualMotorModule armModule;
};
