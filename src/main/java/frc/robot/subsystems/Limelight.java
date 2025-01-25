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
        return (ID >= 1 && ID <= 24) ? ID : 0;
    }

    public double getTA() {
        return LimelightHelpers.getTA("");  // 目標區域
    }

    public double getTX() {
        return LimelightHelpers.getTX(""); 
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    public void updateRobotPose(double x, double y, double angleDegrees) {
        robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }

    double sensorX = robotPose.getX();
    double sensorY = robotPose.getY();

    double px = sensorX; // 機器人的 X 座標
    double py = sensorY; // 機器人的 Y 座標

private boolean isInTriangle(double px, double py, 
double x1, double y1, 
double x2, double y2, 
double x3, double y3) {
// 計算三角形的總面積
double triangleArea = Math.abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0;

// 計算點與三角形三邊形成的小三角形面積
double area1 = Math.abs(px * (y2 - y3) + x2 * (y3 - py) + x3 * (py - y2)) / 2.0;
double area2 = Math.abs(x1 * (py - y3) + px * (y3 - y1) + x3 * (y1 - py)) / 2.0;
double area3 = Math.abs(x1 * (y2 - py) + x2 * (py - y1) + px * (y1 - y2)) / 2.0;

// 如果點在三角形內，三個小面積之和應等於總面積
return (triangleArea == area1 + area2 + area3);
}

@Override
public void periodic() {
// 取得機器人座標
double sensorX = robotPose.getX();
double sensorY = robotPose.getY();
double sensorAngle = robotPose.getRotation().getDegrees();

// 障礙物 1 的三角形頂點
double obstacle1X1 = 3.0, obstacle1Y1 = 3.75;
double obstacle1X2 = 6.0, obstacle1Y2 = 2.5;
double obstacle1X3 = 6.0, obstacle1Y3 = 5.0;

// 障礙物 2 的三角形頂點
double obstacle2X1 = 11.0, obstacle2Y1 = 2.5;
double obstacle2X2 = 11.0, obstacle2Y2 = 5.0;
double obstacle2X3 = 14.0, obstacle2Y3 = 3.75;

// 檢查機器人是否在障礙物 1 或障礙物 2 內
boolean isInObstacle1 = isInTriangle(sensorX, sensorY, 
           obstacle1X1, obstacle1Y1, 
           obstacle1X2, obstacle1Y2, 
           obstacle1X3, obstacle1Y3);

boolean isInObstacle2 = isInTriangle(sensorX, sensorY, 
           obstacle2X1, obstacle2Y1, 
           obstacle2X2, obstacle2Y2, 
           obstacle2X3, obstacle2Y3);
if(ID == 20){
    if (isInObstacle1 || isInObstacle2) {
    // 如果機器人在障礙物內，改變路徑避免障礙物
    System.out.println("機器人進入障礙物區域，改變方向！");
    if (sensorY < obstacle1Y1) {
    sensorY -= 0.5; // 從障礙物上方繞過
    } else {
    sensorY += 0.5; // 從障礙物下方繞過
    }
    } else {
    // 繼續向目標移動的邏輯
    double targetX = 16.25; // 目標 X 坐標
    double targetY = 7.5;  // 目標 Y 坐標
    double deltaX = targetX - sensorX;
    double deltaY = targetY - sensorY;

    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    if (distance > 0.1) {
    double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
    sensorAngle = angleToTarget;

    sensorX += speed * Math.cos(Math.toRadians(sensorAngle));
    sensorY += speed * Math.sin(Math.toRadians(sensorAngle));
    }
}
if(ID == 2){
    if (isInObstacle1 || isInObstacle2) {
    // 如果機器人在障礙物內，改變路徑避免障礙物
    System.out.println("機器人進入障礙物區域，改變方向！");
    if (sensorY < obstacle1Y1) {
    sensorY -= 0.5; // 從障礙物上方繞過
    } else {
    sensorY += 0.5; // 從障礙物下方繞過
    }
    } else {
    // 繼續向目標移動的邏輯
    double targetX = 8.125; // 目標 X 坐標
    double targetY = 7.5;  // 目標 Y 坐標
    double deltaX = targetX - sensorX;
    double deltaY = targetY - sensorY;

    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    if (distance > 0.1) {
    double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
    sensorAngle = angleToTarget;

    sensorX += speed * Math.cos(Math.toRadians(sensorAngle));
    sensorY += speed * Math.sin(Math.toRadians(sensorAngle));
    }
}
}else{
    if (isInObstacle1 || isInObstacle2) {
        // 如果機器人在障礙物內，改變路徑避免障礙物
        System.out.println("機器人進入障礙物區域，改變方向！");
        if (sensorY < obstacle1Y1) {
        sensorY -= 0.5; // 從障礙物上方繞過
        } else {
        sensorY += 0.5; // 從障礙物下方繞過
        }
        } else {
        // 繼續向目標移動的邏輯
        double targetX = 0.5; // 目標 X 坐標
        double targetY = 0.5;  // 目標 Y 坐標
        double deltaX = targetX - sensorX;
        double deltaY = targetY - sensorY;
    
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    
        if (distance > 0.1) {
        double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));
        sensorAngle = angleToTarget;
    
        sensorX += speed * Math.cos(Math.toRadians(sensorAngle));
        sensorY += speed * Math.sin(Math.toRadians(sensorAngle));
            }
        }
    }
// 限制機器人座標範圍，確保不超出比賽場地
sensorX = Math.max(0.5, Math.min(16.25, sensorX));
sensorY = Math.max(0.5, Math.min(7.5, sensorY));

// 更新機器人位置
updateRobotPose(sensorX, sensorY, sensorAngle);

// 更新 Field2d 上的機器人位置
field2d.setRobotPose(robotPose);

// 更新 Shuffleboard 數據
SmartDashboard.putNumber("Robot X", robotPose.getX());
SmartDashboard.putNumber("Robot Y", robotPose.getY());
SmartDashboard.putNumber("Robot Angle", robotPose.getRotation().getDegrees());
    }   
}
}
