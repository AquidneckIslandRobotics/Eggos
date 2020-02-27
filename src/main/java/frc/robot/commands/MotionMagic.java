/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake; 

public class MotionMagic extends CommandBase {
  private Chassis m_subsystem;
  private double m_clicks;
  private Intake intake; 
  private boolean firstRun = true; 
  private double driveTimer; 
  /**
   * Creates a new MotionMagic.
   */
  public MotionMagic(Chassis chassis, double distance, Intake intake) {
    addRequirements(chassis, intake);
    this.intake = intake; 
    m_subsystem = chassis;
    m_clicks = (distance * 1564.5); // per inch presumably 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_subsystem.setConfig(m_subsystem._motion_magic);
    m_subsystem.zeroAllSensors();
    driveTimer = Timer.getFPGATimestamp(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.IntakeInward();
    m_subsystem.setSetpoint(m_clicks);
   if(Timer.getFPGATimestamp() > driveTimer + 5)
    firstRun = false; 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.StopIntake();
    m_subsystem.stopDriveMotors();
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return !firstRun && m_subsystem.onTarget(); 

  }
}
