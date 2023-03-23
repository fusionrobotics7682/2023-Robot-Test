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

  CANSparkMax turret = new CANSparkMax(7, MotorType.kBrushless);
  SparkMaxPIDController turretPIDController = turret.getPIDController();
  RelativeEncoder turretEncoder = turret.getEncoder();

  DifferentialDrive drive = new DifferentialDrive(leftMaster,rightMaster);
  AHRS navx = new AHRS();

  DifferentialDrivePoseEstimator differentialDrivePoseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(0.23), new Rotation2d(), 0, 0, new Pose2d());
  PhotonCameraWrapper cameraWrapper = new PhotonCameraWrapper();

  Field2d field2d = new Field2d();
  Pose2d initialPose2d = new Pose2d();
  Pose2d calculatedPose;
  Pose2d target = new Pose2d();


  // Drive Constants
  private final double kPOSITION_2_METER = 2 * 3.14 * 0.0254 * (1 / 10.72);

  // Turret Constants
  private final double K_TURRET_DEGREE_OFFSET = 90;
  private final double TURRET_GEAR_RATIO = 12.0;
  private final double DEGREES_2_POSITION = TURRET_GEAR_RATIO / 360;
  private final double POSITION_2_DEGREES = 360 / TURRET_GEAR_RATIO;

  Joystick joystick = new Joystick(0);

  PowerDistribution powerDistribution = new PowerDistribution(0, ModuleType.kRev);
  SendableChooser<IdleMode> sendableChooser = new SendableChooser<IdleMode>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    calculatedPose = new Pose2d();

    // PID Configs
    turretPIDController.setP(0.0001);
    turretPIDController.setI(0);
    turretPIDController.setD(0);  
    turretPIDController.setIZone(0);
    turretPIDController.setFF(0);
    turretPIDController.setOutputRange(-1, 1);

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
    SmartDashboard.putNumber("Calculated degree setpoint :", calculateTurretSetpointDegree());
    SmartDashboard.putNumber("Turret degree :", turretEncoder.getPosition() * POSITION_2_DEGREES);

    // Unit Convertion
    double leftMeterDistance = leftMaster.getEncoder().getPosition() * kPOSITION_2_METER;
    double rightMasterDistance = rightMaster.getEncoder().getPosition() * kPOSITION_2_METER;

    // Pose Estimation Test : based on has any targets and pose ambiguity
    calculatedPose = differentialDrivePoseEstimator.update(navx.getRotation2d(), leftMeterDistance, rightMasterDistance);
    if(cameraWrapper.photonCamera.getLatestResult().hasTargets() && cameraWrapper.photonCamera.getLatestResult().getBestTarget().getPoseAmbiguity() > 0.5){
      calculatedPose = cameraWrapper.getEstimatedGlobalPose().get().estimatedPose.toPose2d().interpolate(differentialDrivePoseEstimator.getEstimatedPosition(), 0.4);
    }
    field2d.setRobotPose(calculatedPose);
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Drive Test with coeffecient
    drive.arcadeDrive(joystick.getRawAxis(1)*0.6, joystick.getRawAxis(4)*0.6);
    turretPIDController.setReference(calculateTurretPosition(), ControlType.kPosition);
  }

  public double calculateTurretSetpointDegree(){
    double x = target.getX() - calculatedPose.getX();
    double y = target.getY() - calculatedPose.getY();
    return Math.toDegrees(Math.atan2(y, x));
  }

  public double calculateTurretPosition(){
    return (calculateTurretSetpointDegree() - K_TURRET_DEGREE_OFFSET) * DEGREES_2_POSITION;
  }
}
