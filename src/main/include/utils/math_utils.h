#ifndef MATHUTILS_H
#define MATHUTILS_H

#include "units/angular_velocity.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation2d.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

using namespace frc;
using TPS = units::turns_per_second_t;

Rotation2d getAngleFromRobotToTarget(Translation2d target, Translation2d reference, Rotation2d direction);
Translation2d getDeltaTranslation(Translation2d target, Translation2d reference);
TPS getTPSFromDistance(double distance, double y_intercept, double slope);

#endif 