// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  CANSparkMax leftMaster = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax leftSlave = new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax shoulder = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax turret = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax extensible = new CANSparkMax(9, MotorType.kBrushless);

  SparkMaxPIDController turretController = turret.getPIDController();
  SparkMaxPIDController shoulderController = shoulder.getPIDController();
  SparkMaxPIDController extensibleController = extensible.getPIDController();
  RelativeEncoder turretEncoder = turret.getEncoder();
  RelativeEncoder shoulderEncoder = shoulder.getEncoder();
  RelativeEncoder extensibleEncoder = extensible.getEncoder();

  DifferentialDrive drive = new DifferentialDrive(leftMaster,rightMaster);
  AHRS navx = new AHRS();

  DifferentialDrivePoseEstimator differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(Constants.DriveConstants.kinematics, new Rotation2d(), 0, 0, new Pose2d());
  RobotState robotState = new RobotState(new Rotation2d(), 0, 0, new Pose2d());
  Pose3d targetPose = new Pose3d(new Translation3d(1, 0, 1), new Rotation3d());
  Field2d field2d = new Field2d();
  
  Joystick joystick = new Joystick(0);

  double currentLeftMeterDistance;
  double currentRightMeterDistance;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
   
    SmartDashboard.putData("Robot Pose on Field", field2d);

    turretController.setP(0.001);
    turretController.setI(0);
    turretController.setD(0);
    turretController.setIZone(0);
    rightMaster.setInverted(true);

    shoulderEncoder.setPosition(Constants.ShoulderConstants.DEFAULT_ARM_ANGLE * Constants.ShoulderConstants.DEGREES_2_POSITION);
    //differentialDrivePoseEstimator.resetPosition(new Rotation2d(), 0, 0, new Pose2d(new Translation2d(5, 5), new Rotation2d()));

    navx.reset();

    double coeffecient = 0;
    if(DriverStation.getAlliance()==Alliance.Blue){
      coeffecient+=180;
    }

    try{
      differentialDrivePoseEstimator.resetPosition(new Rotation2d(0), 0, 0, new Pose2d(new Translation2d(5, 5), new Rotation2d(Math.toRadians(navx.getRotation2d().getDegrees()+coeffecient))));
      }catch(Exception exception){
        exception.printStackTrace();
      }
    leftMaster.getEncoder().setPosition(0);
    rightMaster.getEncoder().setPosition(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    currentLeftMeterDistance = 2 * Math.PI * Constants.DriveConstants.WHEEL_RADIUS_METER * (leftMaster.getEncoder().getPosition() / Constants.DriveConstants.GEAR_RATIO);
    currentRightMeterDistance = 2 * Math.PI * Constants.DriveConstants.WHEEL_RADIUS_METER * (rightMaster.getEncoder().getPosition() / Constants.DriveConstants.GEAR_RATIO);

    SmartDashboard.putNumber("Left Meter Distance :", currentLeftMeterDistance);
    SmartDashboard.putNumber("Right Meter Distance :", currentRightMeterDistance);
    SmartDashboard.putNumber("Robot Yaw Heading :", navx.getYaw());

    //robotState.update(navx.getRotation2d(), currentLeftMeterDistance, currentRightMeterDistance, turretEncoder.getPosition() * Constants.TurretConstants.POSITION_2_DEGREES, shoulderEncoder.getPosition() * Constants.ShoulderConstants.POSITION_2_DEGREES, );
    differentialDrivePoseEstimator.update(navx.getRotation2d(), currentLeftMeterDistance, currentRightMeterDistance);
    field2d.setRobotPose(differentialDrivePoseEstimator.getEstimatedPosition());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Drive Test with coeffecient
    drive.arcadeDrive(joystick.getRawAxis(1), joystick.getRawAxis(0));
    //turret.set(joystick.getRawAxis(2));
    //turretDrive(joystick.getDirectionDegrees());
  }

  public double calculateShoulderDegrees(){
    Transform3d transform3d = targetPose.minus(robotState.armPositionPose3d());
    return Math.toDegrees(Math.atan(transform3d.getTranslation().getZ() / transform3d.getTranslation().getX()));
  }

  public double calculateTurretDegrees(){
    Transform2d transform2d = targetPose.toPose2d().minus(robotState.robotPositionPose2d());
    return transform2d.getTranslation().getAngle().getDegrees();
  }

  public void turretDrive(double setpointDegrees){
    turretController.setReference(setpointDegrees * Constants.TurretConstants.DEGREES_2_POSITION, ControlType.kPosition);
  }
}
