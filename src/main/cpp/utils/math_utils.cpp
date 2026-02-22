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

TPS getTPSFromDistance(double distance, double y_intercept, double slope) {
    double raw_tps = slope * distance + y_intercept;
    double tps =  raw_tps;

    if (tps < 0) {
        frc::SmartDashboard::PutString("Shooter TPS Warning⚠️: ", "TPS < 0");
        tps = 0;
    }
    else if (tps > 100) {
        frc::SmartDashboard::PutString("Shooter TPS Warning⚠️: ", "TPS > 100");
        tps = 100;
    }
    frc::SmartDashboard::PutNumber("Shooter Raw TPS: ", raw_tps);
    frc::SmartDashboard::PutNumber("Shooter TPS: ", tps);
    return TPS{tps};
}