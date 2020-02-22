/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  public static WPI_TalonFX climbMotor = new WPI_TalonFX(Constants.Climber);
  public static int distanceFT;
  public static int clicksWanted = distanceFT*200;
  public boolean unclimb = false;
  /**
   * Creates a new Climber.
   */
   public Climber() { 
    climbMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

  }
  public void startClimb(){
    climbMotor.set(ControlMode.PercentOutput, .15);
  }

  public void stopClimb(){
    climbMotor.set(ControlMode.PercentOutput, 0);
  }
  public void resetClimb(){
    climbMotor.set(ControlMode.PercentOutput, -.15);
  }
  @Override
  public void periodic() {
    unclimb = SmartDashboard.getBoolean("Prime Climb Reset", unclimb);
    SmartDashboard.putBoolean("UnClimb Boolean", unclimb);
    // This method will be called once per scheduler run
  }
}
