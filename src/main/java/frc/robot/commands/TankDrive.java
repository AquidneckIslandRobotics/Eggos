/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.GenericHID;

public class TankDrive extends CommandBase {
  private Chassis m_chassis;
  private XboxController m_joy;
  private Button m_butY;
  /**
   * Creates a new TankDrive.
   */
  public TankDrive(Chassis chassis, XboxController joystick, Button buty) {
    m_chassis = chassis;
    m_joy = joystick;
    m_butY = buty;
    addRequirements(chassis);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double lSpeed = (Math.abs(m_joy.getY(GenericHID.Hand.kLeft)) < 0.1)?0:m_joy.getY(GenericHID.Hand.kLeft) * -1;
   double rSpeed = (Math.abs(m_joy.getY(GenericHID.Hand.kRight)) < 0.1)?0:m_joy.getY(GenericHID.Hand.kRight) * -1;
    if (!m_butY.get()){
      lSpeed = lSpeed * 0.70;
      rSpeed = rSpeed * 0.70;
      
      //rotation = rotation * 0.5;
    } else {
      rSpeed = lSpeed;//Use left stick for yeet
    }
    m_chassis.tankDrive(lSpeed, rSpeed);
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
