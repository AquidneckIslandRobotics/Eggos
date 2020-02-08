/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.I2C; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color; 
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3; 

public class ColorSensor extends SubsystemBase {
  public final I2C.Port i2cPort = I2C.Port.kOnboard; 
  public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); 
  public static TalonSRX controlPanel = new TalonSRX(Constants.ControlPanel);  
  public Color detectedColor; 
  /**
   * Creates a new ColorSensor.
   */
  public ColorSensor() {

  }

  @Override
  public void periodic() {
    detectedColor = m_colorSensor.getColor(); 
    double IR = m_colorSensor.getIR(); 
    
    SmartDashboard.putNumber("Red", detectedColor.red); 
    SmartDashboard.putNumber("Green", detectedColor.green); 
    SmartDashboard.putNumber("Blue", detectedColor.blue); 
    SmartDashboard.putNumber("IR", IR); 
    SmartDashboard.putString("Color", getDetectedColor()); 
    SmartDashboard.putNumber("Right X", RobotContainer.getRightX()); 
    int proximity = m_colorSensor.getProximity(); 

    // This method will be called once per scheduler run
  }

  public String getDetectedColor() {
    if(detectedColor.blue > .35) {
    return "Blue"; 
    }else if(detectedColor.green > .5 && detectedColor.red < .2){
      return "Green"; 
    } else if(detectedColor.red > .47 && detectedColor.green > .3){
      return "Red"; 
    } else if(detectedColor.green > .53 && detectedColor.red > .29){
      return "Yellow";
    } else {
      return "Unknown"; 
    }
  }
}
