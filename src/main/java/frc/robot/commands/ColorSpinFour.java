/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.ColorSensor;

public class ColorSpinFour extends CommandBase {
  public String dColor = "unknown";
  public double rotations = 0;
  public String sColor = "unknown";
  public boolean inbetween = false;

  /**
   * Creates a new ColorSpinFour.
   */
  public ColorSpinFour() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotations = 0;
    inbetween = false;
    dColor = Robot.m_colorsensor.getDetectedColor();
    sColor = Robot.m_colorsensor.getDetectedColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("rotations", rotations);
    dColor = Robot.m_colorsensor.getDetectedColor();
    Robot.m_colorsensor.controlPanel.set(ControlMode.PercentOutput, .5);
    if (dColor == sColor) {
      if (inbetween == true) {
        rotations += .5;
        inbetween = false;
      }
    } else {
      inbetween = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ColorSensor.controlPanel.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotations >= 4; 
  }
}
