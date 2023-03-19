// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
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
  CANSparkMax gripper = new CANSparkMax(0, MotorType.kBrushless);

  Joystick joystick = new Joystick(0);
  double inputFactor = 1;

  PowerDistribution powerDistribution = new PowerDistribution(0, ModuleType.kRev);
  SendableChooser<IdleMode> sendableChooser = new SendableChooser<IdleMode>();

  @Override
  public void robotInit() {
    sendableChooser.setDefaultOption("CoastGripper", IdleMode.kCoast);
    sendableChooser.addOption("BrakeGripper", IdleMode.kBrake);
    SmartDashboard.putData(sendableChooser);
    
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Voltage", powerDistribution.getVoltage());
    SmartDashboard.putNumber("TotalCurrent", powerDistribution.getTotalCurrent());
    SmartDashboard.putNumber("gripperVoltage", powerDistribution.getCurrent(0));
    SmartDashboard.putNumber("inputFactor", inputFactor);

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
    gripper.set(joystick.getRawAxis(1)*inputFactor);

    IdleMode currentMode = sendableChooser.getSelected();
    gripper.setIdleMode(currentMode);
  }

}
