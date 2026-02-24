// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "utils/math_utils.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

RobotContainer::RobotContainer()
{
    pathplanner::NamedCommands::registerCommand("Shooting", shooter.Shooting([] { return 60_tps; }).WithTimeout(4_s));
    autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);
    frc::Shuffleboard::GetTab("Field").Add("Field", m_Field2d).WithSize(6, 4);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return FieldCentric_Manualdrive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    shooter.SetDefaultCommand(shooter.Stop());
    intake.SetDefaultCommand(intake.Stop());

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );
    
    joystick.X().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return FieldCentricFacingAngle_Manualdrive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithTargetDirection(drivetrain.GetState().Pose.Rotation() + 
                 getAngleFromRobotToTarget(TargetTranslation , drivetrain.GetState().Pose.Translation(),  drivetrain.GetState().Pose.Rotation()));
        })
    );

    joystick.B().OnTrue(drivetrain.RunOnce([this] { drivetrain.ResetPose(frc::Pose2d(0_m, 4.033663_m, frc::Rotation2d(0_deg)));}));

    joystick.Y().WhileTrue(shooter.Shooting([] { return 60_tps; }));
    
    joystick.A().ToggleOnTrue(intake.Intaking([] { return 30_tps; }));

    joystick.POVUp().OnTrue(intake.Lifting(20_tr));

    joystick.RightTrigger().WhileTrue(shooter.Shooting([] { return 60_tps; }));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));
    
    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });

    frc2::RobotModeTriggers::Teleop().OnTrue(
      intake.Lifting(20_tr)   
    );

    frc2::RobotModeTriggers::Teleop().OnFalse(
      intake.Stop()
    );

}

void RobotContainer::TestBindings(){

    joystick.POVUp().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return forwardStraight.WithVelocityX(0.5_mps).WithVelocityY(0_mps);
        })
    );
    joystick.POVDown().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return forwardStraight.WithVelocityX(-0.5_mps).WithVelocityY(0_mps);
        })
    );

    joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected(); 
}

