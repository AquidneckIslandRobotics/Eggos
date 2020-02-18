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
//import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

public class DriveDistanceAuto extends CommandBase {
  public Chassis m_drive; 
  private double m_distance; 
  //private double m_clicks;
  //public double currentVelocity; 
  //public double initialLeftEncoderPosition; 
  //public double initialRightEncoderPosition; 
  /**
   * Creates a new DriveDistanceAuto.
   */
  //public DriveDistanceAuto(Chassis chassis, double distance) {
   // m_drive = drive; 
    //m_distance = distance; //distance must be done in inches 
   // addRequirements(chassis);
   // m_drive = chassis;    
   //  m_clicks = (distance * 3138.8535); // (inch) * (clicks/inch) = click as a final unit 
   // m_drive.resetEncoder();
  private double m_clicks; 
  public double currentVelocity; 
  public double initialLeftEncoderPosition; 
  public double initialRightEncoderPosition;
  /**
   * Creates a new DriveDistanceAuto.
   */
  public DriveDistanceAuto(Chassis drive, double distance) {
    m_drive = drive; 
    m_distance = distance; 
    addRequirements(drive);
    m_clicks = (m_distance * 195.66879); // (inch) * (clicks/inch) = click as a final unit 
    m_drive.resetEncoder();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drive.setConfig(m_drive._leftConfig, m_drive._rightConfig); 
     //m_drive.resetEncoder();
   // initialLeftEncoderPosition = Robot.m_chassis.getLeftEncoder(); 
   // initialRightEncoderPosition = Robot.m_chassis.getRightEncoder(); 
    SmartDashboard.putNumber("Initial Encoder", initialRightEncoderPosition); 
    currentVelocity = 0; 
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_drive.setSetpoint(m_clicks);
    //System.out.println("clicks" + m_clicks); 
    //m_drive.setSetpoint(m_clicks);

    //}
    if(currentVelocity > 0.5) {
      currentVelocity = 0.5; 
    } else {
      double halfClicks = 0.5 * m_distance + initialRightEncoderPosition; 
     // if (halfClicks > RobotContainer.m_chassis.getRightEncoder()) {
      //  currentVelocity = ((Robot.m_chassis.getRightEncoder()-initialRightEncoderPosition)/195.66879) * 0.09+0.2 ; // I dont know why the 0.09 or the .2 is there, maybe someone else does 
    //  } else {
      //  currentVelocity = ((Robot.m_chassis.getRightEncoder()-initialRightEncoderPosition)/195.66879) * -0.09+0.8; //I dont know why this math is here either 
      }
    //}
  //  SmartDashboard.putNumber("Velocity", currentVelocity); 
    //Robot.m_chassis.curvatureDrive(currentVelocity, 0, false); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false; 
   // if (Robot.m_chassis.getRightEncoder() > initialRightEncoderPosition + m_distance) {
      SmartDashboard.putNumber("Velocity", 0); 
      return true; 
 //   }
 //   else {
  //    return false; 
  //  }
  }
} // im just trying to get this to commit

