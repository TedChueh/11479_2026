#include "utils/math_utils.h"

Rotation2d getAngleFromRobotToTarget(Translation2d target, Translation2d reference, Rotation2d botDirection) {
    Translation3d distanceTranslation(units::meter_t((target - reference).X()),units::meter_t((target - reference).Y()), units::meter_t(0));
    Translation3d botDirectionTranslation(units::meter_t(botDirection.Cos()), units::meter_t(botDirection.Sin()), units::meter_t(0));
    
    Eigen::Vector3d distanceVector = distanceTranslation.ToVector();
    Eigen::Vector3d botDirectionVector = botDirectionTranslation.ToVector();

    Eigen::Vector3d crossVector = distanceVector.cross(botDirectionVector);
    double dot = distanceVector.dot(botDirectionVector);

    double angleFromRobotToTarget = -(crossVector.z()/std::abs(crossVector.z())) * 
           std::acos(dot/(distanceTranslation.Distance(Translation3d(0_m, 0_m, 0_m)).value() * botDirectionTranslation.Distance(Translation3d(0_m, 0_m, 0_m)).value()));

    return frc::Rotation2d(units::radian_t(angleFromRobotToTarget));
}

double getDistanceFromRobotToTarget(Translation2d target, Translation2d reference) {
    Translation2d distanceTranslation(units::meter_t((target - reference).X()),units::meter_t((target - reference).Y()));
    return distanceTranslation.Distance(Translation2d(0_m,0_m)).value();
}