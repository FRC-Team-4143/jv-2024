package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.ProtobufSubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Util;
import frc.lib.subsystem.Subsystem;

public class PoseEstimator extends Subsystem {

    private static PoseEstimator instance;

    public static PoseEstimator getInstance() {
        if (instance == null) {
            instance = new PoseEstimator();
        }
        return instance;
    }

    private PeriodicIo io_;
    private Field2d field_;
    private SwerveDrivePoseEstimator odometry_;
    private SwerveDrivePoseEstimator vision_filtered_odometry_;
    private ProtobufSubscriber<Pose2d> vision_subsciber_;
    private ProtobufPublisher<Pose2d> odom_publisher_;
    private ProtobufPublisher<Pose2d> robot_pose_publisher_;

    PoseEstimator() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("WarVision");
        field_ = new Field2d();
        io_ = new PeriodicIo();
        var field_pose_topic = table.getProtobufTopic("vision/pose", Pose2d.proto);
        var robot_odom_topic = table.getProtobufTopic("vision/odom", Pose2d.proto);
        var robot_pose_topic = table.getProtobufTopic("robot/pose", Pose2d.proto);

        vision_subsciber_ = field_pose_topic.subscribe(new Pose2d());
        odom_publisher_ = robot_odom_topic.publish();
        robot_pose_publisher_ = robot_pose_topic.publish();
    }

    @Override
    public void readPeriodicInputs(double timestamp) {
        TimestampedObject<Pose2d> result = vision_subsciber_.getAtomic();
        if (Util.epislonEquals(result.timestamp, timestamp, 1.0)) {
            vision_filtered_odometry_.addVisionMeasurement(result.value, result.timestamp);
        }
    }

    // Make a subscriber, integate vision measurements wpilib method on the new
    // odometry, getLastChange?
    @Override
    public void updateLogic(double timestamp) {
        var drive = SwerveDrivetrain.getInstance();
        io_.pose_ = odometry_.update(drive.getImuYaw(), drive.getModulePositions());
        io_.vision_filtered_pose_ = vision_filtered_odometry_.update(drive.getImuYaw(), drive.getModulePositions());
    }

    @Override
    public void writePeriodicOutputs(double timestamp) {
        odom_publisher_.set(io_.pose_);
    }

    @Override
    public void outputTelemetry(double timestamp) {
        field_.setRobotPose(io_.vision_filtered_pose_);
        SmartDashboard.putData("Field", field_);
        robot_pose_publisher_.set(io_.vision_filtered_pose_);
    }

    @Override
    public void reset() {
        odometry_ = new SwerveDrivePoseEstimator(SwerveDrivetrain.getInstance().kinematics, new Rotation2d(),
                SwerveDrivetrain.getInstance().getModulePositions(), new Pose2d());
        vision_filtered_odometry_ = new SwerveDrivePoseEstimator(SwerveDrivetrain.getInstance().kinematics,
                new Rotation2d(), SwerveDrivetrain.getInstance().getModulePositions(),
                new Pose2d());
        // vision_filtered_odometry.setVisionMeasurementStdDevs()
        io_ = new PeriodicIo();
    }

    @Override
    public LogData getLogger() {
        return io_;
    }

    /**
     * 
     * @return
     */
    public Pose2d getRobotPose() {
        return io_.vision_filtered_pose_;
    }

    public SwerveDrivePoseEstimator getOdometryPose() {
        return vision_filtered_odometry_;
    }

    /**
     * Resets the robot odom and set the robot pose to supplier Pose
     * 
     * @param pose pose to update Robot pose to
     */
    public void setRobotOdometry(Pose2d pose) {
        var drive = SwerveDrivetrain.getInstance();
        odometry_.resetPosition(drive.getImuYaw(), drive.getModulePositions(), pose);
        vision_filtered_odometry_.resetPosition(drive.getImuYaw(), drive.getModulePositions(), pose);
    }

    class PeriodicIo extends LogData {
        Pose2d pose_ = new Pose2d();
        Pose2d vision_filtered_pose_ = new Pose2d();
    }

}
