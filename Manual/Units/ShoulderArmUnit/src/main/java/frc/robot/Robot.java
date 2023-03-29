// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  CANSparkMax shoulderArmMaster = new CANSparkMax(7, MotorType.kBrushless);

  RelativeEncoder shoulderArmEncoder = shoulderArmMaster.getEncoder();

  Joystick joystick = new Joystick(0);

  double initialShoulderArmDegree = -30;
  double shoulderArmDegree;

  private final double GEAR_RATIO = 100;
  private final double GEAR_RATIO_2 = 1.5;
  private final double GENERAL_RATIO = GEAR_RATIO * GEAR_RATIO_2;
  private final double POSITION_2_DEGREES = 360 / GENERAL_RATIO;
  private final double DEGREES_2_POSITION = GENERAL_RATIO / 360;
  
  @Override
  public void robotInit() {
    shoulderArmEncoder.setPosition(initialShoulderArmDegree * DEGREES_2_POSITION);
  }

  @Override
  public void robotPeriodic() {
    shoulderArmDegree = shoulderArmEncoder.getPosition() * POSITION_2_DEGREES;
    SmartDashboard.putNumber("Shoulder Arm degree :", shoulderArmDegree);
    SmartDashboard.putNumber("Shoulder Arm position :", shoulderArmEncoder.getPosition());
  }

  @Override
  public void teleopInit() {
    shoulderArmEncoder.setPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    shoulderArmMaster.set(joystick.getRawAxis(1)*0.6);
  }
}