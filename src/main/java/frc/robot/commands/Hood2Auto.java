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

public class Hood2Auto extends CommandBase {
  Turret turret;
  double targetAngle;

  /**
   * Creates a new Hood2.
   */
  public Hood2Auto(Turret turret, double targetAngle) {
    this.turret = turret;
    this.targetAngle = targetAngle;
    addRequirements(turret);
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
  targetAngle = Constants.hoodLocate[turret.hoodLocate];
  if (Constants.DEBUG) SmartDashboard.putNumber("Curr", hood);
  if (Constants.DEBUG) SmartDashboard.putNumber("Target", targetAngle);
  if (hood > (targetAngle + 1600)) {
    turret.setHoodAngle(-.1);
  }
 else if (hood < (targetAngle - 1600)) {
    turret.setHoodAngle(.1);
    }
    else {
      turret.setHoodAngle(0);
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
    //return false;
  }
}
