package frc.robot;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotState{

    private ArmOdometry armOdometry;
    private DifferentialDrivePoseEstimator differentialDrivePoseEstimator;
    private PhotonCameraWrapper photonCameraWrapper;

    private Pose2d currentPose2d;

    // Starter position with SmartDashboard
    //private Pose2d startPosition = 

    private static RobotState robotState = null;

    public static RobotState getInstance(){
        if(robotState == null){
            robotState = new RobotState(new Rotation2d(), 0, 0, new Pose2d());
        }
        return robotState;
    }

    public RobotState(Rotation2d gyroAngle, double leftEncoderMeter, double rightEncoderMeter, Pose2d pose2d){
        differentialDrivePoseEstimator =
         new DifferentialDrivePoseEstimator(
                                            Constants.DriveConstants.kinematics,
                                            gyroAngle,
                                            leftEncoderMeter,
                                            rightEncoderMeter,
                                            pose2d);
        photonCameraWrapper = new PhotonCameraWrapper();
    }

    public static class PeriodicIO{
        public double turretAngle;
        public double shoulderAngle;
        public double armLength;
        /* Transformations */
    }

    public void update(Rotation2d gyroAngle, double leftDistance, double rightDistance, double turretAngle, double shoulderAngle, double armLength){
        differentialDrivePoseEstimator.update(gyroAngle, leftDistance, rightDistance);
        armOdometry.update(turretAngle, shoulderAngle, armLength);
    }

    // Arm odometry based-on chassis
    public Pose3d robotArmPose3d(){
        return new Pose3d(armPositionPose3d().getX() + robotPositionPose2d().getX(), armPositionPose3d().getY() + robotPositionPose2d().getY(), armPositionPose3d().getZ(), armPositionPose3d().getRotation());
    }

    // Only Arm Odometry
    public Pose3d armPositionPose3d(){
        return armOdometry.getEstimatedPosition();
    }

    // Only Chassis odometry
    public Pose2d robotPositionPose2d(){
        currentPose2d = differentialDrivePoseEstimator.getEstimatedPosition();
        if(photonCameraWrapper.photonCamera.getLatestResult().getBestTarget().getPoseAmbiguity() > 0.5){
            currentPose2d = currentPose2d.interpolate(photonCameraWrapper.getEstimatedGlobalPose().get().estimatedPose.toPose2d(), 0.4);
        }
        return currentPose2d;
    }

    /*  
     *      ODOMETRY
     * Differential Drive Pose Estimator
     * Arm Pose to robot pose(do calculations for arm length and height)
     *
     *  
     *      CONSTRAINTS
     * robot arm shoulder velocity, acceleration
     * differential constraints 
     * 
     * --------
     *      Transformations
     * robot pose -> turret pose
     * turret pose -> robot end effector position
     * 
     * robot end effector position -> target | Transformations
    */

}
