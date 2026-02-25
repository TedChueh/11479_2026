// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <cameraserver/CameraServer.h>
#include <cmath>

#include "Robot.h"
#include "LimelightHelpers.h"

Robot::Robot() {}

void Robot::RobotInit() {
    auto camera = CameraServer::StartAutomaticCapture();
    camera.SetResolution(320, 240);
    camera.SetFPS(60);
}

void Robot::RobotPeriodic() {   
    CommandScheduler::GetInstance().Run();
    m_timeAndJoystickReplay.Update();

    // SmartDashboard Basic Informations Output
    SmartDashboard::PutNumber("Match Time", DriverStation::GetMatchTime().value());
    SmartDashboard::PutNumber("Battery Voltage", RobotController::GetBatteryVoltage().value());

    // Auto Aiming Target Selection
    if (DriverStation::IsDisabled()) {
        auto const allianceColor = DriverStation::GetAlliance();
        if (allianceColor) {
                *allianceColor == DriverStation::Alliance::kRed
                    ? m_container.TargetTranslation = Translation2d{11.915394_m, 4.033663_m} // Red alliance target position
                    : m_container.TargetTranslation = Translation2d{4.625594_m, 4.033663_m}; // Blue alliance target position
        }
    }

    // Limelight Measurement
    auto& drivetrain = m_container.drivetrain;
    auto& vision = m_container.m_vision;
    auto const driveState = drivetrain.GetState();
    auto const pose = driveState.Pose;

    meters_per_second_t translationSpeed = math::hypot(driveState.Speeds.vx, driveState.Speeds.vy);
    m_container.m_Field2d.SetRobotPose(pose);

    if (kUseLimelight) {
        vision.PeriodicUpdate(pose, translationSpeed, driveState.Speeds.omega);
        if(auto meas = vision.GetMeasurement()) {
            if (meas->isReliableForSeeding){
                drivetrain.ResetPose(meas->pose);
            }
            else {
                drivetrain.AddVisionMeasurement(meas->pose, meas->timestamp, {meas->xyStdDev, meas->xyStdDev, meas->rotStdDev});
            }
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

// void Robot::TestInit() {
//     frc2::CommandScheduler::GetInstance().CancelAll();
// }

// void Robot::TestPeriodic() {}

// void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
