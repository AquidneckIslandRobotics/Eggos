/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Distance extends CommandBase {
  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  public double distance;
  //public int y = table.getEntry("ty");
  public int yzero = (int) getLimelightY() + 30 +5;
  //public NetworkTableEntry y = yzero;
    /**
   * Creates a new Distance.
   */
  public double getLimelightX() {
    return tx.getDouble(0);
  }
   
  public double getLimelightY() { 
    return ty.getDouble(0);
  }

  public Distance() {
    double distance = 43.0/Math.tan(yzero);
    //return;
    // int distance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return SmartDashboard.putNumber("Distance", distance);
  }
}
