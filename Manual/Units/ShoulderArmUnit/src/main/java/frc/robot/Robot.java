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
  
  CANSparkMax shoulderArmMaster = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax shoulderArmSlave = new CANSparkMax(0, MotorType.kBrushless);

  RelativeEncoder shoulderArmEncoder = shoulderArmMaster.getEncoder();
  double shoulderArmGearRatio = 1;

  Joystick joystick = new Joystick(0);

  PowerDistribution powerDistribution = new PowerDistribution(0, ModuleType.kRev);
  SendableChooser <IdleMode> sendableChooser = new SendableChooser<IdleMode>();

  double inputFactor = 1;
  double shoulderArmUpLimitDegree = 270;
  double shoulderArmDownLimitDegree = -30;
  double initialShoulderArmDegree = -30;
  double shoulderArmDegree;
  
  @Override
  public void robotInit() {
    shoulderArmSlave.follow(shoulderArmMaster); 
    shoulderArmEncoder.setPosition(shoulderArmGearRatio * initialShoulderArmDegree / 360);

    sendableChooser.setDefaultOption("CoastShoulderArm", IdleMode.kCoast);
    sendableChooser.addOption("BrakeShoulderArm", IdleMode.kBrake);
    SmartDashboard.putData(sendableChooser);
  }

  @Override
  public void robotPeriodic() {

    shoulderArmDegree = (shoulderArmEncoder.getPosition() / shoulderArmGearRatio) * 360;

    SmartDashboard.putNumber("Voltage", powerDistribution.getVoltage());
    SmartDashboard.putNumber("totalCurrent", powerDistribution.getTotalCurrent());
    SmartDashboard.putNumber("shoulderArmMasterVoltage", powerDistribution.getCurrent(0));
    SmartDashboard.putNumber("shoulderArmSlaveVoltage", powerDistribution.getCurrent(1));
    SmartDashboard.putNumber("inputFactor", inputFactor);
    SmartDashboard.putNumber("Shoulder Arm degree :", shoulderArmDegree);


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

  @Override
  public void teleopPeriodic() {

    if(shoulderArmDegree < shoulderArmUpLimitDegree && shoulderArmDegree > shoulderArmDownLimitDegree){
        shoulderArmMaster.set(joystick.getRawAxis(1)*inputFactor);
    }

    IdleMode currentMode = sendableChooser.getSelected();
    shoulderArmMaster.setIdleMode(currentMode);
    shoulderArmSlave.setIdleMode(currentMode);
  }
}