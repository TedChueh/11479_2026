// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include <pathplanner/lib/auto/NamedCommands.h>

#include "Telemetry.h"

class RobotContainer {
private:
    units::meters_per_second_t MaxSpeed = 1.0 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric FieldCentric_Manualdrive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
        .WithDriveRequestType(swerve::DriveRequestType::Velocity); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};
    swerve::requests::RobotCentric forwardStraight = swerve::requests::RobotCentric{}
        .WithDriveRequestType(swerve::DriveRequestType::Velocity);
    swerve::requests::FieldCentricFacingAngle FieldCentricFacingAngle_Manualdrive = swerve::requests::FieldCentricFacingAngle{}
        .WithDeadband(MaxSpeed * 0.05).WithRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
        .WithDriveRequestType(swerve::DriveRequestType::Velocity).WithHeadingPID(8, 0, 0); // Use open-loop control for drive motors

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController joystick{0};

public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};

private:
    /* Path follower */
    frc::SendableChooser<frc2::Command *> autoChooser;

public:
    RobotContainer();
    frc2::Command *GetAutonomousCommand();
    frc::Field2d m_Field2d;
    frc::Translation2d TargetTranslation{}; 

private:
    void ConfigureBindings();

    IntakeSubsystem intake{
        1, 2,
        61, 62,
        DualMotorModule::Config{
            .motorRightInvert = true,
            .motorLeftInvert = false,
            .kS = 0.25,
            .kV = 0.101,
            .kA = 0.0,
            .kP = 0.0,
            .kI = 0,
            .kD = 0,
            .PeakVoltage = 12_V,
            .PeakCurrent = 40_A,
            .RampPeriod = 0.5_s
        },
        DualMotorModule::Config{
            .motorRightInvert = true,
            .motorLeftInvert = false,
            .kS = 0.195,
            .kV = 0.108,
            .kA = 0.0,
            .kP = 0.5,
            .kI = 0,
            .kD = 0,
            .PeakVoltage = 12_V,
            .PeakCurrent = 40_A,
            .RampPeriod = 0.5_s,
            .MotionMagicCruiseVelocity = 20_tps,
            .MotionMagicAcceleration   = 20_tr_per_s_sq
        }
    };

    ShooterSubsystem shooter{
        51, 52,
        54,
        55,
        DualMotorModule::Config{
            .motorRightInvert = true,
            .motorLeftInvert = false,
            .kS = 0.19,
            .kV = 0.111,
            .kA = 0.0,
            .kP = 0.015,
            .kI = 0,
            .kD = 0,
            .PeakVoltage = 12_V,
            .PeakCurrent = 40_A,
            .RampPeriod = 0.5_s
        },
        SingleMotorModule::Config{
            .motorInvert = false,
            .kS = 0.197,
            .kV = 0.11,
            .kA = 0.0,
            .kP = 0.01,
            .kI = 0,
            .kD = 0,
            .PeakVoltage = 12_V,
            .PeakCurrent = 40_A,
            .RampPeriod = 0.5_s
        },
        SingleMotorModule::Config{
            .motorInvert = false,
            .kS = 0.2,
            .kV = 0,
            .kA = 0.0,
            .kP = 0.,
            .kI = 0,
            .kD = 0,
            .PeakVoltage = 12_V,
            .PeakCurrent = 40_A,
            .RampPeriod = 0.1_s
        }
    };

    // DualMotorModule::Config m_testConfig{
    // .motorRightInvert = true,
    // .motorLeftInvert  = false,

    // .kS = 0.25,
    // .kV = 0.101,
    // .kA = 0.0,
    // .kP = 0.0,
    // .kI = 0.0,
    // .kD = 0.0,

    // .PeakVoltage = 8_V,
    // .PeakCurrent = 40_A,
    // .RampPeriod  = 0.5_s,

    // };

    // DualMotorModule m_testModule{1, 2, m_testConfig};
};
