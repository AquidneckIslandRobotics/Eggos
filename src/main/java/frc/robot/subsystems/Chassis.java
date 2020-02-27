/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  private PigeonIMU pidgey;
  private TalonFX leftLead;
  private BaseMotorController leftFollow;
  private TalonFX rightLead;
  private BaseMotorController rightFollow;

  // public Encoder rightEncoder = new Encoder(Constants.EncoderRA, Constants.EncoderRB);
  // public Encoder leftEncoder = new Encoder(Constants.EncoderLA, Constants.EncoderLB);
  public TalonFXConfiguration _motion_magic = new TalonFXConfiguration();

  public TalonFXConfiguration _leftConfig = new TalonFXConfiguration(); 
  public TalonFXConfiguration _rightConfig = new TalonFXConfiguration(); 
	
  private ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();

  /**
   * Creates a new Chassis.
   */
  public Chassis() {
    leftLead = new TalonFX(14);
    leftFollow = new TalonFX(15);
    rightLead = new TalonFX(1);
    rightFollow = new TalonFX(16);
    pidgey = new PigeonIMU(20)

    leftFollow.configFactoryDefault();
    leftFollow.follow(leftLead);
    rightFollow.configFactoryDefault();
    rightFollow.follow(rightLead);

    leftLead.configFactoryDefault();
    rightLead.configFactoryDefault();
    pidgey.configFactoryDefault();

    leftLead.setInverted(TalonFXInvertType.Clockwise);
    leftLead.setSensorPhase(false);
    leftFollow.setInverted(TalonFXInvertType.Clockwise);
    rightLead.setInverted(TalonFXInvertType.CounterClockwise);
    rightLead.setSensorPhase(false);
    rightFollow.setInverted(TalonFXInvertType.CounterClockwise);

    leftLead.configOpenloopRamp(0.8);
    leftFollow.configOpenloopRamp(0.8);
    rightLead.configOpenloopRamp(0.8);
    rightFollow.configOpenloopRamp(0.8);


    leftLead.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, 30);
    leftLead.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, 30);
    leftLead.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 30);

    rightLead.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, 30);
    rightLead.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, 30);
    rightLead.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, 30);
    rightLead.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, 30);
    rightLead.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, 30);
  
    pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 05, 30);

    _motion_magic.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
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

    _leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; 
    _rightConfig.remoteFilter0.remoteSensorDeviceID = leftLead.getDeviceID(); 
    _rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; 
    
    // Setup Left (Aux) and Right (Master)
    _rightConfig.sum0Term = FeedbackDevice.RemoteSensor0;    //Aux Selected Sensor
    _rightConfig.sum1Term = FeedbackDevice.IntegratedSensor; //Local IntegratedSensor
    _rightConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorSum; //Sum0 + Sum1
    _rightConfig.primaryPID.selectedFeedbackCoefficient = 0.5;

    // setRobotConfigs(_rightInvert, _rightConfig); 
    _rightConfig.slot0.kF = Constants.kGains_Distanc.kF; 
    _rightConfig.slot0.kP = Constants.kGains_Distanc.kP;
    _rightConfig.slot0.kI = Constants.kGains_Distanc.kI;
    _rightConfig.slot0.kD = Constants.kGains_Distanc.kD;
    _rightConfig.slot0.integralZone = Constants.kGains_Distanc.kIzone;
    _rightConfig.slot0.closedLoopPeakOutput = Constants.kGains_Distanc.kPeakOutput;
    // Heading Config
    _rightConfig.remoteFilter1.remoteSensorDeviceID = pidgey.getDeviceID();    //Pigeon Device ID
    _rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw; //This is for a Pigeon over CAN
    _rightConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1; //Set as the Aux Sensor
    _rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 3600.0 / Constants.kPigeonUnitsPerRotation;

    _rightConfig.auxPIDPolarity = false;

    _rightConfig.slot1.kF = Constants.kGains_Turning.kF;
    _rightConfig.slot1.kP = Constants.kGains_Turning.kP;
    _rightConfig.slot1.kI = Constants.kGains_Turning.kI;
    _rightConfig.slot1.kD = Constants.kGains_Turning.kD;
    _rightConfig.slot1.integralZone = Constants.kGains_Turning.kIzone;
    _rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_Turning.kPeakOutput;

    _leftConfig.neutralDeadband = Constants.kNeutralDeadband;
    _rightConfig.neutralDeadband = Constants.kNeutralDeadband;
	  
    int closedLoopTimeMs = 1;
    rightLead.configClosedLoopPeriod(0, closedLoopTimeMs, 30);
    rightLead.configClosedLoopPeriod(1, closedLoopTimeMs, 30);

    _rightConfig.motionAcceleration = 6000; //(distance units per 100 ms) per second
    _rightConfig.motionCruiseVelocity = 15000; //distance units per 100 ms

    leftLead.configAllSettings(_leftConfig);
    rightLead.configAllSettings(_rightConfig); 

    rightLead.selectProfileSlot(0,0);
    rightLead.selectProfileSlot(1,1);
	  
    // Music
    _instruments.add(leftLead);
    _instruments.add((TalonFX)leftFollow);
    _instruments.add(rightLead);
    _instruments.add((TalonFX)rightFollow);
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Lead Direction", leftLead.getInverted());
    SmartDashboard.putBoolean("Right Lead Direction", rightLead.getInverted());

    SmartDashboard.putNumber("Left Encoder Count", leftLead.getSelectedSensorPosition()); 
    SmartDashboard.putNumber("Right Encoder COUNT", rightLead.getSelectedSensorPosition()); 

    SmartDashboard.putNumber("ENCODER Encoder Count", leftLead.getSelectedSensorVelocity()); 

    SmartDashboard.putBoolean("On Target", onTarget()); 
    SmartDashboard.putNumber("Target Error", leftLead.getClosedLoopError()); 
    // This method will be called once per scheduler run


  }

  public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
    double leftSpeed = speed - rotation;//(quickTurn ? -rotation : speed - (speed != 0 ? rotation : 0));
    double rightSpeed = speed + rotation;//(quickTurn ? rotation : speed + (speed != 0 ? rotation : 0));
    leftLead.set(ControlMode.PercentOutput, leftSpeed);
    rightLead.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void setConfig(TalonFXConfiguration config) {
    leftLead.configAllSettings(config);
    leftFollow.follow(leftLead); 
    leftFollow.configNeutralDeadband(0); 
    rightLead.follow(leftLead);
    rightLead.configNeutralDeadband(0);
    rightFollow.follow(leftLead); 
    rightFollow.configNeutralDeadband(0); 
  }
  public void setConfig(TalonFXConfiguration leftConfig, TalonFXConfiguration rightConfig) {
    leftLead.configAllSettings(leftConfig); 
    rightLead.configAllSettings(rightConfig); 
    }
  
  public double getRightEncoder() {
    return rightLead.getSelectedSensorPosition();
  }

  public double getLeftEncoder() {
    return leftLead.getSelectedSensorPosition();
  }

public void stopDriveMotors() {
  leftLead.set(ControlMode.PercentOutput, 0); 
  rightLead.set(ControlMode.PercentOutput, 0); 
}



  public void setSetpoint(double setpoint) {
    System.out.println("Out: " + leftLead.getMotorOutputPercent());
    System.out.println("Cur: " + leftLead.getSelectedSensorPosition(0));
    System.out.println("Vel: " + leftLead.getSelectedSensorVelocity(0));
    System.out.println("Err: " + leftLead.getClosedLoopError(0));
    System.out.println("Trg: " + setpoint);

    leftLead.set(ControlMode.MotionMagic, setpoint);
   // rightLead.set(ControlMode.MotionMagic, setpoint); 
  }
	
  public void setSetpoint(double distance, double heading) {
    // assumes distance is in clicks
    rightLead.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, heading);
    leftLead.follow(rightLead, FollowerType.AuxOutput1);
    rightFollow.follow(rightLead);
    leftFollow.follow(rightLead);
  }

  public boolean onTarget() {
    return Math.abs(leftLead.getClosedLoopError()) < 1000;
  }

  public int getError() {
    return leftLead.getClosedLoopError();
  }

  public void resetEncoder(){
    leftLead.getSensorCollection().setIntegratedSensorPosition(0, 30); 
    rightLead.getSensorCollection().setIntegratedSensorPosition(0, 30); 
  }
	
  public void zeroAllSensors() {
    leftLead.getSensorCollection().setIntegratedSensorPosition(0, 30); 
    rightLead.getSensorCollection().setIntegratedSensorPosition(0, 30); 
    pidgey.setYaw(0, 30);
    pidgey.setAccumZAngle(0, 30);
  }
	
  public double getAngle() {
    //double[] ypr = new double[3];
    //pidgey.getYawPitchRoll(ypr);
    return rightLead.getSelectedSensorPosition(1);
  }
	
  public void switchDirection(){
   
    leftLead.setInverted(!leftLead.getInverted());
    leftFollow.setInverted(!leftFollow.getInverted());
    rightLead.setInverted(!rightLead.getInverted());
    rightFollow.setInverted(!rightFollow.getInverted());
    
  }
  public void stop() {
    leftLead.set(ControlMode.PercentOutput, 0); 
    rightLead.set(ControlMode.PercentOutput, 0); 
  }
	
  public ArrayList<TalonFX> getInstruments() {
	return _instruments;	  
  }
}
// " hey buddy if you could just switch these motors to the other dirction that'd be great"

/* if(programmers == tired){
  println("NAP"); 
 }else {
   println("Keep programming"); 
 }; */
