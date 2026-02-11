#ifndef MATHUTILS_H
#define MATHUTILS_H
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation2d.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
using namespace frc;

Rotation2d getAngleFromRobotToTarget(Translation2d target, Translation2d reference, Rotation2d direction);
double getDistanceFromRobotToTarget(Translation2d target, Translation2d reference);

#endif 