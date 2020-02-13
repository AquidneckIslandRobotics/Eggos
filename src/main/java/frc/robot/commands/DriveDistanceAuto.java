/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class DriveDistanceAuto extends CommandBase {
  private Chassis m_drive; 
  private double m_distance; 
  private double m_clicks; 
  /**
   * Creates a new DriveDistanceAuto.
   */
  public DriveDistanceAuto(Chassis drive, double distance) {
    m_drive = drive; 
    m_distance = distance; 
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_clicks = ((m_distance * 12.0)/18.8) * 2048.0; // I HAVE NO CLUE IF THIS MATH IS RIGHT BUT WHATEVER 
    System.out.println("clicks" + m_clicks); 
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
