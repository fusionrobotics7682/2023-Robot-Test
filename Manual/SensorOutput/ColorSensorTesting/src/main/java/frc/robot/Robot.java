// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private AHRS navx = new AHRS();
  private ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard);
  private Encoder encoder = new Encoder(6,7);

  private ColorMatch colorMatch = new ColorMatch();

  private Color red = new Color(255, 0, 0);
  private Color blue = new Color(0, 0, 255);
  private Color green = new Color(0, 255, 0);
  private Color purple = new Color(0.22, 0.33, 0.445);
  private final Color yellow = new Color(0.40, 0.55, 0.057);
  private String currentColor = "";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    colorMatch.addColorMatch(purple);
    colorMatch.addColorMatch(yellow);
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

    Color color = colorSensor.getColor();
    SmartDashboard.putNumber("Navx getAngle() :", navx.getAngle());
    SmartDashboard.putNumber("Navx angle adjustment :", navx.getAngleAdjustment());
    SmartDashboard.putNumber("Navx getCompassheadind()", navx.getCompassHeading());
    SmartDashboard.putNumber("R", color.red);
    SmartDashboard.putNumber("G", color.green);
    SmartDashboard.putNumber("B", color.blue);
    SmartDashboard.putNumber("Navx getYaw()", navx.getYaw());
    double value = 360/1024;
    SmartDashboard.putNumber("Angle :", 0.3515 * encoder.getDistance());
    SmartDashboard.putNumber("Raw encoder value :", encoder.getDistance());
    
    if(colorMatch.matchClosestColor(color).color == purple){
      currentColor = "purple";
    }
    else if(colorMatch.matchClosestColor(color).color == yellow){
      currentColor = "yellow";
    }
    else{
      currentColor = "NONE";
    }
    SmartDashboard.putString("Current color", currentColor);
  }

}
