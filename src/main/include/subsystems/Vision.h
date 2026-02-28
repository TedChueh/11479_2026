#pragma once

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/geometry/Pose2d.h> 
#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>           
#include <units/velocity.h>         
#include <units/angular_velocity.h> 
#include <units/time.h>             
#include <units/angle.h>            
#include <optional>   
#include "LimelightHelpers.h"

using namespace std;
using namespace frc;
using namespace frc2;
using namespace units;

struct VisionMeasurement {
    Pose2d pose;
    double xyStdDev;
    double rotStdDev;
    int tagCount;
    second_t timestamp;
    bool isReliableForSeeding; 
};

class VisionSubsystem : public SubsystemBase {
public:
    VisionSubsystem() = default;

    void PeriodicUpdate(
        const Pose2d& robotPose, 
        const meters_per_second_t translationSpeed, 
        const degrees_per_second_t angularVelocity
    );

    optional<VisionMeasurement> GetMeasurement() const { return m_measurement; }

private:
    optional<VisionMeasurement> m_measurement;
};