// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
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

  CANSparkMax turret = new CANSparkMax(6, MotorType.kBrushless);
  RelativeEncoder turretEncoder = turret.getEncoder();

  Joystick joystick = new Joystick(0);
  double inputFactor = 0.5;

  double turretGearRatio = 12;
  double initialTurretDegrees = 0;
  double turretUpLimitDegrees = 180;
  double turretDownLimitDegrees = -180;

  double currentTurretDegrees;

  final double POSITION_2_DEGREES = 12 * 360;

  // PowerDistribution powerDistribution = new PowerDistribution(0, ModuleType.kRev);
  SendableChooser <IdleMode> sendableChooser = new SendableChooser<IdleMode>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    sendableChooser.setDefaultOption("CoastTurret", IdleMode.kCoast);
    sendableChooser.addOption("BrakeTurret", IdleMode.kBrake);
    SmartDashboard.putData(sendableChooser);
    turretEncoder.setPosition(initialTurretDegrees);
    turret.setIdleMode(IdleMode.kCoast);
    turret.setInverted(true);
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
    currentTurretDegrees = (turretEncoder.getPosition() / POSITION_2_DEGREES);

    // SmartDashboard.putNumber("Voltage", powerDistribution.getVoltage());
    // SmartDashboard.putNumber("totalCurrent", powerDistribution.getTotalCurrent());
    // SmartDashboard.putNumber("turretVoltage", powerDistribution.getCurrent(1));
    SmartDashboard.putNumber("inputFactor", inputFactor);
    SmartDashboard.putNumber("Turret Degree", currentTurretDegrees);
  }

  @Override
  public void teleopPeriodic() {
    
    if(joystick.getRawButton(1)) {turret.set(0.2);}
    
    else if(joystick.getRawButton(2)) {turret.set(0.5);}
   
    else if(joystick.getRawButton(3)) {turret.set(0.7);}
    
    else if(joystick.getRawButton(4)) {turret.set(1);}

    else{
      turret.set(0);
    }
    
  }

}
