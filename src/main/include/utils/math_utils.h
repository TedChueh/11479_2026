#ifndef MATHUTILS_H
#define MATHUTILS_H

#include "units/angular_velocity.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation2d.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

using namespace frc;
using TPS = units::turns_per_second_t;

Rotation2d calcHeadingError(Translation2d targetPosition, Translation2d robotPosition, Rotation2d robotDirection);
Translation2d calcRelativeTranslationToTarget(Translation2d targetPosition, Translation2d referencePosition);
TPS getTPSFromDistance(double distance, double y_intercept, double slope);
// Rotation2d calcVelocityCompAngle(double shootAngle, double deltaHeight, Translation2d targetPosition, Translation2d robotPosition, ChassisSpeeds robotVelocity);

#endif 