// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
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
};
