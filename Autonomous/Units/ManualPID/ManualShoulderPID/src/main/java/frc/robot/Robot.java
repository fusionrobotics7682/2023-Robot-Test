// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  CANSparkMax shoulderCanSparkMax = new CANSparkMax(0, MotorType.kBrushless);
  SparkMaxPIDController pidController = shoulderCanSparkMax.getPIDController();
  RelativeEncoder encoder = shoulderCanSparkMax.getEncoder();

  private final Joystick joystick = new Joystick(0);

  private final double GEAR_RATIO = 100;
  private final double GEAR_RATIO_2 = 1.5;
  private final double GENERAL_RATIO = GEAR_RATIO * GEAR_RATIO_2;
  private final double POSITION_2_DEGREES = 360 / GENERAL_RATIO;
  private final double DEGREES_2_POSITION = GENERAL_RATIO / 360;
  private final double beginingPositionDegrees = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Beginning Position
    encoder.setPosition(DEGREES_2_POSITION * beginingPositionDegrees);

    // PID Config
    pidController.setP(0.0001);
    pidController.setI(0);
    pidController.setD(0);
    pidController.setIZone(0);
    pidController.setFF(0);
    pidController.setOutputRange(-1, 1);
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
    SmartDashboard.putNumber("Position", encoder.getPosition());
    SmartDashboard.putNumber("Shoulder Degree :", encoder.getPosition() * POSITION_2_DEGREES);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    manualControl(joystick.getRawAxis(1));
    pidControl(Math.abs(joystick.getDirectionDegrees()));
  }

  public void manualControl(double speed){
    shoulderCanSparkMax.set(speed);
    //pidController.setReference(speed, ControlType.kDutyCycle);
  }

  public void pidControl(double degreeSetpoint){
    double positionSetpoint = degreeSetpoint * DEGREES_2_POSITION;
    pidController.setReference(positionSetpoint, ControlType.kPosition);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
