/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;

public class Hood2 extends CommandBase {
  Turret turret;
  double targetAngle;

  /**
   * Creates a new Hood2.
   */
  public Hood2(Turret turret, double targetAngle) {
    this.turret = turret;
    this.targetAngle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  double hood = turret.getHoodAngle();
  
  SmartDashboard.putNumber("Curr", hood);
  SmartDashboard.putNumber("Target", targetAngle);
  if (hood == targetAngle) {
    turret.setHoodAngle(0);
  }
  else if (hood > (targetAngle + 200)) {
    turret.setHoodAngle(-.05);
  }
  if (hood < (targetAngle - 250)) {
    turret.setHoodAngle(.05);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setHoodAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((turret.getHoodAngle() > (targetAngle - 500)) && (turret.getHoodAngle() < (targetAngle + 500))) return true;
    else return false;
  }
}
