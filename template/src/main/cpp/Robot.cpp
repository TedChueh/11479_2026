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
}

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

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
