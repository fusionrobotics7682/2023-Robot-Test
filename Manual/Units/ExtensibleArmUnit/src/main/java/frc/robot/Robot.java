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

  CANSparkMax extensibleArmMaster = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax extensibleArmSlave = new CANSparkMax(1, MotorType.kBrushless);

  Joystick joystick = new Joystick(0);

  PowerDistribution powerDistribution = new PowerDistribution(0, ModuleType.kRev);
  SendableChooser <IdleMode> sendableChooser = new SendableChooser<IdleMode>();

  double inputFactor = 1;
  // Limit in neo encoder position units
  double extensiblePositionLimit = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    extensibleArmSlave.follow(extensibleArmMaster);

    sendableChooser.setDefaultOption("CoastExtenSibleArm", IdleMode.kCoast);
    sendableChooser.addOption("BrakeExtensibleArm", IdleMode.kBrake);
    SmartDashboard.putData(sendableChooser);
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
    SmartDashboard.putNumber("voltage",powerDistribution.getVoltage() );
    SmartDashboard.putNumber("totalCurrent", powerDistribution.getTotalCurrent());
    SmartDashboard.putNumber("extensibleArmMasterVoltage",powerDistribution.getCurrent(0));
    SmartDashboard.putNumber("extensibleArmSlave",powerDistribution.getCurrent(1));
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

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    extensibleArmMaster.set(joystick.getRawAxis(1)*inputFactor);

    if(Math.abs(extensibleArmMaster.getEncoder().getPosition()) > extensiblePositionLimit){
      extensibleArmMaster.set(0);
    }

    IdleMode currentMode = sendableChooser.getSelected();
    extensibleArmMaster.setIdleMode(currentMode);
    extensibleArmSlave.setIdleMode(currentMode);
  }

}
