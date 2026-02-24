// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "LimelightHelpers.h"

#include <cameraserver/CameraServer.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

Robot::Robot() {}

void Robot::RobotInit() {
    auto camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(320, 240);
    camera.SetFPS(60);
}

void Robot::RobotPeriodic() {   
    m_timeAndJoystickReplay.Update();
    frc2::CommandScheduler::GetInstance().Run();

    frc::SmartDashboard::PutNumber(
    "Match Time",frc::DriverStation::GetMatchTime().value());
    
    frc::SmartDashboard::PutNumber(
        "Battery Voltage",
        frc::RobotController::GetBatteryVoltage().value());

    if (frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? m_container.TargetTranslation = frc::Translation2d{11.915394_m, 4.033663_m} // Red alliance target position
                    : m_container.TargetTranslation = frc::Translation2d{4.625594_m, 4.033663_m}; // Blue alliance target position
        }
    }

    auto const driveState = m_container.drivetrain.GetState();
    auto const pose  = driveState.Pose;
    auto const heading = driveState.Pose.Rotation().Degrees();
    auto const omega = driveState.Speeds.omega;
    m_container.m_Field2d.SetRobotPose(pose);

    if (kUseLimelight && !frc::DriverStation::IsAutonomousEnabled()) {
        LimelightHelpers::SetRobotOrientation("limelight", heading.value(), 0, 0, 0, 0, 0);
        auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (llMeasurement && llMeasurement->tagCount > 0 && units::math::abs(omega) < 2_tps) {
            m_container.drivetrain.AddVisionMeasurement(llMeasurement->pose, llMeasurement->timestampSeconds);
        }
    }
}

// void Robot::DisabledInit() {}

// void Robot::DisabledPeriodic() {}

// void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand);
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
