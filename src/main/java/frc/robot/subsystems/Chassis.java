/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Chassis extends SubsystemBase {
  private TalonFX leftLead;
  private BaseMotorController leftFollow;
  private TalonFX rightLead;
  private BaseMotorController rightFollow;

  public TalonFXConfiguration _motion_magic = new TalonFXConfiguration(); 
  
  /**
   * Creates a new Chassis.
   */
  public Chassis() {
    leftLead = new TalonFX(14);
    leftFollow = new TalonFX(15);
    rightLead = new TalonFX(1);
    rightFollow = new TalonFX(16);


    leftFollow.configFactoryDefault();
    leftFollow.follow(leftLead);
    rightFollow.configFactoryDefault();
    rightFollow.follow(rightLead);

    leftLead.configFactoryDefault();
    rightLead.configFactoryDefault();

    leftLead.setInverted(true);
    leftLead.setSensorPhase(false);
    leftFollow.setInverted(true);
    rightLead.setInverted(false);
    rightLead.setSensorPhase(false);
    rightFollow.setInverted(false);

    leftLead.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
    leftLead.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

    rightLead.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30); 
    rightLead.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30); 


    _motion_magic.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    _motion_magic.neutralDeadband = 0.001;
    _motion_magic.nominalOutputForward = 0;
    _motion_magic.nominalOutputReverse = 0;
    _motion_magic.peakOutputForward = 1;
    _motion_magic.peakOutputReverse = -1;
    _motion_magic.slot0.kF = 0.2;
    _motion_magic.slot0.kP = 0.2;
    _motion_magic.slot0.kI = 0.0;
    _motion_magic.slot0.kD = 0.0;
    _motion_magic.motionAcceleration = 6000;
    _motion_magic.motionCruiseVelocity = 15000;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Lead Direction", leftLead.getInverted()); 
    SmartDashboard.putBoolean("Right Lead Direction", rightLead.getInverted()); 
    // This method will be called once per scheduler run
  }

  public void curvatureDrive(double speed, double rotation, boolean quickTurn){
    double leftSpeed = (quickTurn?-rotation:speed - (speed != 0?rotation:0));
    double rightSpeed = (quickTurn?rotation:speed + (speed != 0?rotation:0));

    leftLead.set(ControlMode.PercentOutput,leftSpeed);
    rightLead.set(ControlMode.PercentOutput,rightSpeed);
  }

  public void setConfig(TalonFXConfiguration config) {
    leftLead.configAllSettings(config);
  }

  public void setSetpoint(double setpoint) {
    System.out.println("Out: " + leftLead.getMotorOutputPercent());
    System.out.println("Cur: " + leftLead.getSelectedSensorPosition(0));
    System.out.println("Vel: " + leftLead.getSelectedSensorVelocity(0));
    System.out.println("Err: " + leftLead.getClosedLoopError(0));
    System.out.println("Trg: " + setpoint);
    leftLead.set(ControlMode.MotionMagic, setpoint);
  }

  public void resetEncoder(){
    leftLead.setSelectedSensorPosition(0);
  }
  public void switchDirection(){
   
    leftLead.setInverted(!leftLead.getInverted());
    leftFollow.setInverted(!leftFollow.getInverted());
    rightLead.setInverted(!rightLead.getInverted());
    rightFollow.setInverted(!rightFollow.getInverted());
    
  }
}
// " hey buddy if you could just switch these motors to the other dirction that'd be great"

/* if(programmers == tired){
  println("NAP"); 
 }else {
   println("Keep programming"); 
 }; */