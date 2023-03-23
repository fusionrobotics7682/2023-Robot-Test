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
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
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
  CANSparkMax leftMaster = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax leftSlave = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax rightMaster = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax rightSlave = new CANSparkMax(3, MotorType.kBrushless);

  DifferentialDrive drive = new DifferentialDrive(leftMaster,rightMaster);
  AHRS navx = new AHRS();

  DifferentialDrivePoseEstimator differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(0.23), new Rotation2d(), 0, 0, new Pose2d());
  PhotonCameraWrapper cameraWrapper = new PhotonCameraWrapper();
  Field2d field2d = new Field2d();
  Pose2d calculatedPose;

  private final double kPOSITION_2_METER = 2 * 3.14 * 0.0254 * (1 / 10.72);

  Joystick joystick = new Joystick(0);

  PowerDistribution powerDistribution = new PowerDistribution(0, ModuleType.kRev);
  SendableChooser<IdleMode> sendableChooser = new SendableChooser<IdleMode>();
  double inputFactor = 1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    calculatedPose = new Pose2d();

    // Add chooser to SmartDashboard
    sendableChooser.setDefaultOption("CoastDrive", IdleMode.kCoast);
    sendableChooser.addOption("BrakeDrive", IdleMode.kBrake);
    SmartDashboard.putData(sendableChooser);    
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
    // General Information about the Robot

    // Monitoring pose ambiguity
    cameraWrapper.photonCamera.getLatestResult().targets.stream().forEach(x -> {
      SmartDashboard.putNumber("Target pose ambiguity :", x.getPoseAmbiguity());
    });
    SmartDashboard.putNumber("Best target pose ambiguity :", cameraWrapper.photonCamera.getLatestResult().getBestTarget().getPoseAmbiguity());
    SmartDashboard.putBoolean("Vision has target? : ", cameraWrapper.photonCamera.getLatestResult().hasTargets());

    // Unit Convertion
    double leftMeterDistance = leftMaster.getEncoder().getPosition() * kPOSITION_2_METER;
    double rightMasterDistance = rightMaster.getEncoder().getPosition() * kPOSITION_2_METER;

    // Pose Estimation Test : based on has any targets and pose ambiguity
    calculatedPose = differentialDrivePoseEstimator.update(navx.getRotation2d(), leftMeterDistance, rightMasterDistance);
    if(cameraWrapper.photonCamera.getLatestResult().hasTargets() && cameraWrapper.photonCamera.getLatestResult().getBestTarget().getPoseAmbiguity() > 0.5){
      calculatedPose = cameraWrapper.getEstimatedGlobalPose().get().estimatedPose.toPose2d().interpolate(differentialDrivePoseEstimator.getEstimatedPosition(), 0.4);
    }
    field2d.setRobotPose(calculatedPose);

    // Input Factor for differential drive
    if(joystick.getRawButton(1)){
      inputFactor = 1;
    }
    else if(joystick.getRawButton(2)){
      inputFactor = 0.75;
    }
    else if(joystick.getRawButton(3)){
      inputFactor = 0.50; 
    }
    else if(joystick.getRawButton(4)){
      inputFactor = 0.25;
    }
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Drive Test with coeffecient
    drive.arcadeDrive(joystick.getRawAxis(1)*inputFactor, joystick.getRawAxis(4)*inputFactor);
  }
}
