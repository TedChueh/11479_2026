#include "subsystems/Vision.h"
#include <frc/geometry/Rotation2d.h>
#include <cmath>

void VisionSubsystem::PeriodicUpdate(const Pose2d& robotPose, const meters_per_second_t translationSpeed, const degrees_per_second_t angularVelocity) {
    // 提供陀螺儀資訊給 Limelight (MegaTag2 的核心要求)
    LimelightHelpers::SetRobotOrientation(
        "limelight",
        robotPose.Rotation().Degrees().value(),
        angularVelocity.value(),
        0, 0, 0, 0
    );

    // 取得 MegaTag2 估計值
    auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // 基本檢查：沒看到 Tag 就清空數據
    if (!llMeasurement || llMeasurement->tagCount < 1) {
        m_measurement.reset();
        return;
    }

    // 更新頻率限制 至少100ms
    static second_t lastVisionUpdate = 0_s;
    second_t currentTimestamp = second_t(llMeasurement->timestampSeconds);

    if (currentTimestamp - lastVisionUpdate < 0.1_s) {
        m_measurement.reset();
        return;
    }
    lastVisionUpdate = currentTimestamp;
    
    // --- 進入過濾階段 ---
    double dist = llMeasurement->avgTagDist;
    int tagCount = llMeasurement->tagCount;

    // A. 距離過濾
    if (meter_t{dist} > 4.0_m) {
        m_measurement.reset();
        return;
    }

    // B. 速度過濾
    bool isAuto = DriverStation::IsAutonomous();
    meters_per_second_t maxSpeed = isAuto ? 1_mps : 2.0_mps; 
    if (translationSpeed > maxSpeed || math::abs(angularVelocity) > 360_deg_per_s) {
        m_measurement.reset();
        return;
    }

    // C. 誤差跳變與重設判定
    auto poseError = llMeasurement->pose.Translation().Distance(robotPose.Translation());
    
    //【關鍵邏輯】判斷這筆數據是否具備「強制重設 (Seed)」的資格
    // 條件：誤差大、機器人幾乎靜止、且看到多個 Tag 以確保位置絕對正確
    bool canForceSeed = (poseError > 0.7_m && translationSpeed < 0.1_mps && tagCount >= 2);

    // 如果誤差大於 0.5 公尺，但又不符合「強制重設」的條件，這筆數據就有問題，丟棄它
    if (poseError > 0.5_m && !canForceSeed) {
        m_measurement.reset();
        return;
    }

    // 計算動態信任權重 (Standard Deviations)
    double xyStdDev;
    double rotStdDev = 999999.0; // 始終不信任視覺旋轉，交給 Pigeon 2

    if (tagCount >= 2) {
        xyStdDev = 0.2 + (dist * 0.15);
    } else {
        xyStdDev = 0.6 + (dist * 0.4);
    }

    // 打包數據
    VisionMeasurement vm;
    vm.pose = llMeasurement->pose;
    vm.xyStdDev = xyStdDev;
    vm.rotStdDev = rotStdDev;
    vm.tagCount = tagCount;
    vm.timestamp = currentTimestamp;
    
    // --- 新增：將判定結果傳給底盤 ---
    vm.isReliableForSeeding = canForceSeed; 

    m_measurement = vm; 

    // Telemetry
    SmartDashboard::PutNumber("Vision/PoseError_m", poseError.value());
    SmartDashboard::PutNumber("Vision/XY_StdDev", xyStdDev);
    SmartDashboard::PutNumber("Vision/TagsSeen", (double)tagCount);
    SmartDashboard::PutBoolean("Vision/IsReliableForSeed", canForceSeed);
}