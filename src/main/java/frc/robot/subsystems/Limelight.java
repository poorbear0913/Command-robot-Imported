package frc.robot.subsystems;

import javax.print.attribute.standard.MediaSize.NA;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
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

    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

    private final PoseEstimator m_PoseEstimator = new PoseEstimator<>(null, null, null, null)

    boolean doRejectUpdate = false;

    if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1){
      if(mt1.rawFiducials[0].ambiguity > .7)
      {
        doRejectUpdate = true;
      }
      if(mt1.rawFiducials[0].distToCamera > 3)
      {
        doRejectUpdate = true;
      }
    }
    if(mt1.tagCount == 0)
    {
      doRejectUpdate = true;
    }

    if(!doRejectUpdate)
    {
        m_poseEstimate.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        m_poseEstimate.addVisionMeasurement(
          mt1.pose,
          mt1.timestampSeconds);
    }

    private NetworkTable table = NetworkTableInstance.getDefault().getTable(LimelightConstants.Name);
    private Field2d field2d;
    private Pose2d robotPose;

    private double ID;
    private double tA;  // 目標區域 (百分比)
    private double tX;  // 目標水平偏移量 (度)

    private double speed = 0.1;        // 基本前進速度
    private double angularSpeed = 15.0;   // 基本旋轉速度

    public Limelight() {
        field2d = new Field2d();
        robotPose = new Pose2d(0.5, 0.5, new Rotation2d(0.0)); // 初始位置
        SmartDashboard.putData("field2d", field2d);
    }

    public double getFiducialID() {
        double ID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
        SmartDashboard.putNumber("Fiducial ID", ID);
        return (ID >= 1 && ID <= 24) ? ID : 0;
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
        SmartDashboard.putString("Robot Pose", robotPose.toString());
    }

    public void periodic() {
        // 從 Limelight 取得 ID, 目標區域與目標偏移
        ID = getFiducialID();
        tA = getTA();
        tX = getTX();

        // 使用 Limelight 數據來調整機器人位置
        double sensorX = robotPose.getX();
        double sensorY = robotPose.getY();
        double sensorAngle = robotPose.getRotation().getDegrees();

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