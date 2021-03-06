/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class TurretLimelight extends CommandBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  Turret turret;
  int count;
  /**
   * Creates a new TurretLimelight.
   */
  public TurretLimelight(Turret turret) {
    this.turret = turret;
    //addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.turret.lightsOn();
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = turret.getLimelightX();
    count++;
    if (y > 0 + .5) {
      turret.setSpeed(-.3);
      if (Constants.DEBUG) SmartDashboard.putString("direction", "right");
    }
    else if (y < 0 - 0.5) {
      turret.setSpeed(.3);
      if (Constants.DEBUG) SmartDashboard.putString("direction", "left");
    }
    else {
      turret.stopTurret();
      if (Constants.DEBUG) SmartDashboard.putString("direction", "straight");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopTurret();
    turret.lightsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // if (Math.abs(turret.getLimelightX()) < 1 && count > 50)
    //return true;
     return false; 
  }
}
