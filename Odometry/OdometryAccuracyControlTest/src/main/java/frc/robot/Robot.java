// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  CANSparkMax leftMaster = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax leftSlave = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(5, MotorType.kBrushless);

  DifferentialDrive drive = new DifferentialDrive(leftMaster,rightMaster);
  AHRS navx = new AHRS();

  DifferentialDrivePoseEstimator differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(0.23), new Rotation2d(), 0, 0, new Pose2d());
  PhotonCameraWrapper cameraWrapper = new PhotonCameraWrapper();

  Field2d field2d = new Field2d();

  Pose2d calculatedPose;

  final double TURRET_GEAR_RATIO = 12;
  final double TURRET_DEGREE_TO_POSITION = 12 / 360;
  final double CHASSIS_GEAR_RATIO = 10.72;
  final double WHEEL_DIAMETER = Units.inchesToMeters(3.1);

  private final double kPOSITION_2_METER = 2 * 3.14 * WHEEL_DIAMETER * (1 / CHASSIS_GEAR_RATIO);

  Joystick joystick = new Joystick(0);
  double inputFactor = 1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    rightMaster.setInverted(true);

    SmartDashboard.putData("Robot Pose on Field", field2d);
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
    // Unit Convertion
    double leftMeterDistance = leftMaster.getEncoder().getPosition() * kPOSITION_2_METER;
    double rightMasterDistance = rightMaster.getEncoder().getPosition() * kPOSITION_2_METER;

    SmartDashboard.putNumber("Left Meter distance :", leftMeterDistance);
    SmartDashboard.putNumber("Right Meter distance :", rightMasterDistance);
    SmartDashboard.putBoolean("Vision has Target :", cameraWrapper.photonCamera.getLatestResult().hasTargets());

    try{  
    calculatedPose = differentialDrivePoseEstimator.update(navx.getRotation2d(), leftMeterDistance, rightMasterDistance);
    field2d.setRobotPose(calculatedPose);
    }catch(Exception exception){
      exception.printStackTrace();
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    navx.reset();

    double coeffecient = 0;
    if(DriverStation.getAlliance()==Alliance.Blue){
      coeffecient+=180;
    }

    try{
        differentialDrivePoseEstimator.resetPosition(new Rotation2d(0), 0, 0, new Pose2d(new Translation2d(), new Rotation2d(Math.toRadians(navx.getRotation2d().getDegrees()+coeffecient))));
      }catch(Exception exception){
        exception.printStackTrace();
      }
    leftMaster.getEncoder().setPosition(0);
    rightMaster.getEncoder().setPosition(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Drive Test with coeffecient
    SmartDashboard.putNumber("Navx rotation2d getDegrees", navx.getRotation2d().getDegrees());
    drive.arcadeDrive(joystick.getRawAxis(1)*0.6, joystick.getRawAxis(2)*0.6); 
  }
}
