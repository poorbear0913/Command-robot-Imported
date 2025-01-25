package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightConstants.Name);
    private Field2d field2d;
    private Pose2d robotPose;

    private double ID;
    private double tA;  // 目標區域 (百分比)
    private double tX;  // 目標水平偏移量 (度)

    private double speed = 0.1;        // 基本前進速度
    private double angularSpeed = 15.0;  // 基本旋轉速度

    public Limelight() {
        field2d = new Field2d();
        robotPose = new Pose2d(0.5, 0.5, new Rotation2d(0.0)); // 初始位置
        SmartDashboard.putData("field2d", field2d);
    }

    public double getFiducialID() {
        double ID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
        return (ID >= 1 && ID <= 16) ? ID : 0;
    }

    public double getTA() {
        return LimelightHelpers.getTA("");  // 目標區域
    }

    public double getTX() {
        return LimelightHelpers.getTX("");  // 目標水平偏移量
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public void updateRobotPose(double x, double y, double angleDegrees) {
        robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }

    public void periodic() {
        // 從 Limelight 取得 ID, 目標區域與目標偏移
        ID = getFiducialID();
        tA = getTA();
        tX = getTX();
    
        // 假設右下角的目標位置
        double targetX = 15.37;  // 右下角的 X 坐標
        double targetY = 0.3;  // 右下角的 Y 坐標

        // 使用 Limelight 數據來調整機器人位置
        double sensorX = robotPose.getX();
        double sensorY = robotPose.getY();
        double sensorAngle = robotPose.getRotation().getDegrees();
    
        // 如果偵測到 AprilTag 1，將車子移動到右下角
        if (ID == 2) {
            double deltaX = targetX - sensorX;  // 計算機器人與目標 X 之間的差距
            double deltaY = targetY - sensorY;  // 計算機器人與目標 Y 之間的差距
    
            // 計算移動速度，這裡可以根據距離做調整
            double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);  // 兩點之間的距離
            speed = 0.5;  // 設定前進的速度
    
            if (distance > 0.1) {  // 如果距離大於閾值，繼續移動
                double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));  // 計算面向目標的角度
                sensorAngle = angleToTarget;  // 更新機器人角度為目標角度
    
                // 根據目標位置更新機器人位置
                sensorX += speed * Math.cos(Math.toRadians(sensorAngle));  // 根據角度移動 X 坐標
                sensorY += speed * Math.sin(Math.toRadians(sensorAngle));  // 根據角度移動 Y 坐標
            }
        } else if (ID == 20) {
            double targetX1 = 16.25;  // 右上角的 X 坐標
            double targetY1 = 7.5;  // 右上角的 Y 坐標
            double deltaX1 = targetX1 - sensorX;  // 計算機器人與目標 X 之間的差距
            double deltaY1 = targetY1 - sensorY;  // 計算機器人與目標 Y 之間的差距
    
            // 計算移動速度，這裡可以根據距離做調整
            double distance = Math.sqrt(deltaX1 * deltaX1 + deltaY1 * deltaY1);  // 兩點之間的距離
            speed = 0.5;  // 設定前進的速度
    
            if (distance > 0.1) {  // 如果距離大於閾值，繼續移動
                double angleToTarget = Math.toDegrees(Math.atan2(deltaY1, deltaX1));  // 計算面向目標的角度
                sensorAngle = angleToTarget;  // 更新機器人角度為目標角度
    
                // 根據目標位置更新機器人位置
                sensorX += speed * Math.cos(Math.toRadians(sensorAngle));  // 根據角度移動 X 坐標
                sensorY += speed * Math.sin(Math.toRadians(sensorAngle));  // 根據角度移動 Y 坐標
            }
        }else{ 
            double targetX2 = 0.5;  // 右上角的 X 坐標
            double targetY2 = 0.5;  // 右上角的 Y 坐標
            double deltaX2 = targetX2 - sensorX;  // 計算機器人與目標 X 之間的差距
            double deltaY2 = targetY2 - sensorY;  // 計算機器人與目標 Y 之間的差距
    
            // 計算移動速度，這裡可以根據距離做調整
            double distance = Math.sqrt(deltaX2 * deltaX2 + deltaY2 * deltaY2);  // 兩點之間的距離
            speed = 0.5;  // 設定前進的速度
    
            if (distance > 0.1) {  // 如果距離大於閾值，繼續移動
                double angleToTarget = Math.toDegrees(Math.atan2(deltaY2, deltaX2));  // 計算面向目標的角度
                sensorAngle = angleToTarget;  // 更新機器人角度為目標角度
    
                // 根據目標位置更新機器人位置
                sensorX += speed * Math.cos(Math.toRadians(sensorAngle));  // 根據角度移動 X 坐標
                sensorY += speed * Math.sin(Math.toRadians(sensorAngle));  // 根據角度移動 Y 坐標
            }
            
        }
        
    
        // 限制機器人坐標範圍，確保不超出比賽場地
        sensorX = Math.max(0.5, Math.min(16.25, sensorX));
        sensorY = Math.max(0.5, Math.min(7.5, sensorY));
        double Y_A = 0.5;
        double Y_B = 1;
        double X_A = 14.5;
        double X_B = 16.25;
        double m = (Y_B - Y_A) / (X_B - X_A);
        double b = Y_A - m * X_A;  // 計算截距

        if (sensorX >= X_A && sensorY <= (m * sensorX + b)) {
        // 機器人超過斜牆，調整角度或移動方向
        sensorAngle += 180;  // 例如，讓它反彈
        }

        // 更新機器人位置
        updateRobotPose(sensorX, sensorY, sensorAngle);
    
        // 更新 Field2d 上的機器人位置
        field2d.setRobotPose(robotPose);
    
        // 更新 Shuffleboard 上的數據顯示
        SmartDashboard.putNumber("Tag_ID", ID);
        SmartDashboard.putNumber("Tag_Area", tA);
        SmartDashboard.putNumber("Tag_X", tX);
        SmartDashboard.putNumber("Robot X", robotPose.getX());
        SmartDashboard.putNumber("Robot Y", robotPose.getY());
        SmartDashboard.putNumber("Robot Angle", robotPose.getRotation().getDegrees());
    }
}