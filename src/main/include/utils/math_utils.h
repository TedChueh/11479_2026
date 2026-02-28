#ifndef MATHUTILS_H
#define MATHUTILS_H

#include "units/angular_velocity.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <cmath>

using namespace std;
using namespace frc;
using namespace units;
using TPS = turns_per_second_t;

constexpr double g = 9.81;

Rotation2d calcHeadingError(Translation2d targetPosition, Translation2d robotPosition, Rotation2d robotDirection);
double calcRelativeDistanceToTarget(Translation2d targetPosition, Translation2d referencePosition);
TPS getTPSFromDistance(double distance, double y_intercept, double slope);
Rotation2d calcVelocityCompAngle(double shootDegree, double deltaHeight, Translation2d targetPosition, Translation2d robotPosition, ChassisSpeeds robotVelocity);

#endif 