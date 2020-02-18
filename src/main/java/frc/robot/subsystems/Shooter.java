/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  
  private static WPI_TalonFX shooterWheel1 = new WPI_TalonFX(Constants.LeftShooter);
  private static WPI_TalonFX shooterWheel2 = new WPI_TalonFX(Constants.RightShooter);
  CANSparkMax hopperRight = new CANSparkMax(Constants.HopperRight, MotorType.kBrushless);
  CANSparkMax hopperLeft = new CANSparkMax(Constants.HopperLeft, MotorType.kBrushless);
  CANSparkMax feed = new CANSparkMax(Constants.Feed, MotorType.kBrushless);
  /**
   * Creates a new Shooter.
   */
  public Shooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("L Hopper", hopperLeft.get());
    SmartDashboard.putNumber("R Hopper", hopperRight.get());
  }
  public void HopperIntake() {
    hopperRight.set(0.5);
    hopperLeft.set(-0.5);
    feed.set(.5);
  }
  public void HopperOuttake() {
    hopperRight.set(-0.5);
    hopperLeft.set(0.5);
  }
  public void stopHopper(){
    hopperRight.set(0);
    hopperLeft.set(0);
    feed.set(0);
  }
  public void startWheel() {
    shooterWheel1.set(ControlMode.PercentOutput, -0.85);
    shooterWheel2.set(ControlMode.PercentOutput, 0.85);
  }
  public void stopWheel() {
    shooterWheel1.set(ControlMode.PercentOutput, 0);
    shooterWheel2.set(ControlMode.PercentOutput, 0);
  }
}