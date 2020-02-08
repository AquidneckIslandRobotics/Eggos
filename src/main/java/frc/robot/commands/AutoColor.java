/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot; 

public class AutoColor extends CommandBase {
  public String dColor = "unknown"; 
  public String chosenColor = "unknown"; 

  /**
   * Creates a new AutoColor.
   */
  public AutoColor(String color) {
    this.chosenColor = color; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dColor = Robot.m_colorsensor.getDetectedColor(); 
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dColor = Robot.m_colorsensor.getDetectedColor(); 
    SmartDashboard.putString("Auto Detected Color", dColor); 
    Robot.m_colorsensor.controlPanel.set(ControlMode.PercentOutput, .2); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_colorsensor.controlPanel.set(ControlMode.PercentOutput, 0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return dColor == chosenColor;
  }
}
