// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/events/EventTrigger.h>

#include "RobotContainer.h"

using namespace pathplanner;

RobotContainer::RobotContainer()
{ 
    // NamedCommands::registerCommand("myCommand", command);
    // EventTrigger("myCommand").WhileTrue(command);

    autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    SmartDashboard::PutData("Auto Mode", &autoChooser);
    Shuffleboard::GetTab("Field").Add("Field", m_Field2d).WithSize(6, 4);

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Subsystem Default Command
    drivetrain.SetDefaultCommand(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return FieldCentric_Manualdrive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed)                            // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate);               // Drive counterclockwise with negative X (left)
        })
    );

    // Disable Mode Trigger
    RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );
    
    // Joystick Binding
    // joystick.X().WhileTrue(command);
    // joystick.X().ToggleOnTrue(command);
    // joystick.X().OnTrue(command); 

    // Register Telemetry
    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
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
        return point.WithModuleDirection(Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
}

Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected(); 
}

