// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  PhotonCameraWrapper cameraWrapper = new PhotonCameraWrapper();
  DifferentialDrivePoseEstimator differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(0.23), new Rotation2d(), 0, 0, new Pose2d());
  Field2d field2d = new Field2d();

  Joystick joystick = new Joystick(0);
  double inputFactor = 1;

  double gearRatio = 10.72;
  double wheelRadiusMeter = Units.inchesToMeters(3);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    rightMaster.setInverted(true);
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
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

    double leftMeterDistance = 2 * 3.14 * wheelRadiusMeter * (leftMaster.getEncoder().getPosition() / gearRatio);
    double rightMasterDistance = 2 * 3.14 * wheelRadiusMeter * (rightMaster.getEncoder().getPosition() / gearRatio);
    SmartDashboard.putNumber("Left meter encoder :", leftMeterDistance);
    SmartDashboard.putNumber("Right meter encoder :", rightMasterDistance);
    try{
    differentialDrivePoseEstimator.update(navx.getRotation2d(), leftMeterDistance, rightMasterDistance);
    field2d.setRobotPose(differentialDrivePoseEstimator.getEstimatedPosition());}
    catch(Exception exception){
      exception.printStackTrace();
    }
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Trial 1
    //differentialDrivePoseEstimator.resetPosition(new Rotation2d(0), 0, 0, new Pose2d(new Translation2d(), new Rotation2d()));
    // Trial 2
    try{
    differentialDrivePoseEstimator.resetPosition(new Rotation2d(0), 0, 0, cameraWrapper.getEstimatedGlobalPose().get().estimatedPose.toPose2d());
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
    drive.arcadeDrive(joystick.getRawAxis(1)*0.6, -joystick.getRawAxis(2)*0.6);
  }
}
