/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis;

public class DriveDistanceAuto extends CommandBase {
  private Chassis m_drive; 
  private double m_distance; 
  private double m_clicks;
  public double currentVelocity; 
  public double initialLeftEncoderPosition; 
  public double initialRightEncoderPosition; 
  /**
   * Creates a new DriveDistanceAuto.
   */
  public DriveDistanceAuto(Chassis chassis, double distance) {
   // m_drive = drive; 
    //m_distance = distance; //distance must be done in inches 
    addRequirements(chassis);
    m_drive = chassis;    
     m_clicks = (distance * 3138.8535); // (inch) * (clicks/inch) = click as a final unit 
   // m_drive.resetEncoder();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setConfig(m_drive._leftConfig, m_drive._rightConfig); 
     m_drive.resetEncoder();

    // I HAVE NO CLUE IF THIS MATH IS RIGHT BUT WHATEVER //Math is in git hub under frc robot command driveAuto i believe.
    
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // m_drive.setSetpoint(m_clicks);
    //System.out.println("clicks" + m_clicks); 
    m_drive.setSetpoint(m_clicks);

    }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
    
  }
}
