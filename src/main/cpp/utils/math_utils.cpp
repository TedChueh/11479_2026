#include "utils/math_utils.h"

Rotation2d calcHeadingError(Translation2d targetPosition, const impl::SwerveDrivetrainImpl::SwerveDriveState& robotState) {
    Translation2d robotPosition  = robotState.Pose.Translation();
    Rotation2d    robotDirection = robotState.Pose.Rotation();
    Translation2d toTarget = targetPosition - robotPosition;
    Translation2d botDirVec(meter_t(robotDirection.Cos()), meter_t(robotDirection.Sin()));
    
    double cross = botDirVec.X().value() * toTarget.Y().value() - botDirVec.Y().value() * toTarget.X().value();
    double dot   = botDirVec.X().value() * toTarget.X().value() + botDirVec.Y().value() * toTarget.Y().value();
    double angle = atan2(cross, dot);

    return Rotation2d(radian_t(angle));
}

double calcRelativeDistanceToTarget(Translation2d targetPosition, Translation2d referencePosition) {
    Translation2d diff = targetPosition - referencePosition;
    return diff.Norm().value();
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

Rotation2d calcVelocityCompAngle(degree_t shootDegree,
                                 meter_t deltaHeight,
                                 Translation2d targetPosition,
                                 const impl::SwerveDrivetrainImpl::SwerveDriveState& robotState) {

    Translation2d robotPosition  = robotState.Pose.Translation();
    ChassisSpeeds robotVelocity  = robotState.Speeds;

    Translation2d targetVector   = targetPosition - robotPosition;
    meter_t targetDistance       = targetVector.Norm();

    double shootRad = shootDegree.to<double>() * M_PI / 180.0;

    meter_t denom = 2.0 * (targetDistance * tan(shootRad) - deltaHeight);
    if (denom <= 0_m) {
        SmartDashboard::PutString("Velocity Comp Angle Warning⚠️: ", "Denominator <= 0");
        return Rotation2d(0_rad);
    }

    meters_per_second_t desiredVx{
        std::sqrt((g_accel * targetDistance * targetDistance / denom).value())
    };

    double TVcrossRVV = targetVector.X().value() * robotVelocity.vy.value() -
                        targetVector.Y().value() * robotVelocity.vx.value();
    double tangentialSpeed = TVcrossRVV / targetDistance.value();

    double compAngleRad = atan2(tangentialSpeed, desiredVx.value());

    return Rotation2d(radian_t(compAngleRad));
}
