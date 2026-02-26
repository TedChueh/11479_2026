#include "utils/math_utils.h"

Rotation2d calcHeadingError(Translation2d targetPosition, Translation2d robotPosition, Rotation2d robotDirection) {
    Translation2d toTarget = targetPosition - robotPosition;
    Translation2d botDirVec(meter_t(robotDirection.Cos()), meter_t(robotDirection.Sin()));
    
    double cross = botDirVec.X().value() * toTarget.Y().value() - botDirVec.Y().value() * toTarget.X().value();
    double dot   = botDirVec.X().value() * toTarget.X().value() + botDirVec.Y().value() * toTarget.Y().value();
    double angle = atan2(cross, dot);

    return Rotation2d(units::radian_t(angle));
}

Translation2d calcRelativeTranslationToTarget(Translation2d targetPosition, Translation2d referencePosition) {
    return targetPosition - referencePosition;
}

TPS getTPSFromDistance(double distance, double y_intercept, double slope) {
    double raw_tps = slope * distance + y_intercept;
    double tps =  raw_tps;

    if (tps < 0) {
        SmartDashboard::PutString("Shooter TPS Warning⚠️: ", "TPS < 0");
        tps = 0;
    }
    else if (tps > 100) {
        SmartDashboard::PutString("Shooter TPS Warning⚠️: ", "TPS > 100");
        tps = 100;
    } 
    else {
        SmartDashboard::PutString("Shooter TPS Warning⚠️: ", "TPS within bounds");
    }
    SmartDashboard::PutNumber("Shooter Raw TPS: ", raw_tps);
    SmartDashboard::PutNumber("Shooter TPS: ", tps);
    return TPS{tps};
}