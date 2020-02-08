/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.GenericHID;

public class Drive extends CommandBase {
  Chassis m_subsystem;
  XboxController m_joystick;
  Button m_button, m_buttonY;

  /**
   * Creates a new Drive.
   */
  public Drive(Chassis subsystem, XboxController joy, Button butt, Button butY) {
    m_subsystem = subsystem;
    m_joystick = joy;
    m_button = butt;
    m_buttonY = butY;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = (Math.abs(m_joystick.getY(GenericHID.Hand.kLeft)) < 0.1)?0:m_joystick.getY(GenericHID.Hand.kLeft);
    double rotation = (Math.abs(m_joystick.getX(GenericHID.Hand.kRight)) < 0.1)?0:m_joystick.getX(GenericHID.Hand.kRight) *-1;
    if (!m_buttonY.get()){
      speed = speed * 0.5;
      //rotation = rotation * 0.5;
    }
    m_subsystem.curvatureDrive(speed, rotation, m_button.get());
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
