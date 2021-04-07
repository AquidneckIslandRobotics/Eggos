/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.*;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  private PigeonIMU pidgey;
  private WPI_TalonFX leftLead, leftFollow, rightLead, rightFollow;
  private int dir = 1;
  private boolean shootFront = true;

  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private static NetworkTableEntry tx = table.getEntry("tx");
  private static NetworkTableEntry ty = table.getEntry("ty");
  private static NetworkTableEntry ta = table.getEntry("ta");

  // public Encoder rightEncoder = new Encoder(Constants.EncoderRA, Constants.EncoderRB);
  // public Encoder leftEncoder = new Encoder(Constants.EncoderLA, Constants.EncoderLB);
  public TalonFXConfiguration _motion_magic = new TalonFXConfiguration();

  public TalonFXConfiguration _leftConfig = new TalonFXConfiguration(); 
  public TalonFXConfiguration _rightConfig = new TalonFXConfiguration(); 
	
  private ArrayList<WPI_TalonFX> _instruments = new ArrayList<WPI_TalonFX>();

  // The robot's drive
  private final DifferentialDrive m_drive;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new Chassis.
   */
  public Chassis() {
    leftLead = new WPI_TalonFX(14);
    leftFollow = new WPI_TalonFX(15);
    rightLead = new WPI_TalonFX(1);
    rightFollow = new WPI_TalonFX(16);
    pidgey = new PigeonIMU(0);
    m_drive = new DifferentialDrive(leftLead, rightLead);

    leftFollow.configFactoryDefault();
    leftFollow.follow(leftLead);
    rightFollow.configFactoryDefault();
    rightFollow.follow(rightLead);

    leftLead.configFactoryDefault();
    rightLead.configFactoryDefault();
    pidgey.configFactoryDefault();

    leftLead.setInverted(TalonFXInvertType.Clockwise);
    //leftLead.setSensorPhase(false);
    leftFollow.setInverted(TalonFXInvertType.Clockwise);
    rightLead.setInverted(TalonFXInvertType.CounterClockwise);
    //rightLead.setSensorPhase(false);
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

    _rightConfig.motionAcceleration = 7000; //(distance units per 100 ms) per second
    _rightConfig.motionCruiseVelocity = 15000; //distance units per 100 ms

    leftLead.configAllSettings(_leftConfig);
    rightLead.configAllSettings(_rightConfig); 

    rightLead.selectProfileSlot(0,0);
    rightLead.selectProfileSlot(1,1);
	  
    // Music
    _instruments.add(leftLead);
    _instruments.add((WPI_TalonFX)leftFollow);
    _instruments.add(rightLead);
    _instruments.add((WPI_TalonFX)rightFollow);

    //odometry?
    m_odometry = new DifferentialDriveOdometry(new Rotation2d(getAngle()));
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Lead Direction", leftLead.getInverted());
    SmartDashboard.putBoolean("Right Lead Direction", rightLead.getInverted());

    SmartDashboard.putNumber("Left Encoder Count", leftLead.getSelectedSensorPosition()); 
    SmartDashboard.putNumber("Right Encoder COUNT", rightLead.getSelectedSensorPosition()); 

    SmartDashboard.putNumber("LEncoder Count", leftLead.getSelectedSensorVelocity()); 
    SmartDashboard.putNumber("REncoder Count", rightLead.getSelectedSensorVelocity());

    SmartDashboard.putBoolean("On Target", onTarget()); 
    SmartDashboard.putNumber("Target Error", leftLead.getClosedLoopError()); 

    SmartDashboard.putNumber("Heading", getAngle());

    SmartDashboard.putBoolean("Shooter Front", shootFront);
    SmartDashboard.putBoolean("Intake Front", !shootFront);
    // This method will be called once per scheduler run

    //more odometry
    m_odometry.update(new Rotation2d(getAngle()), leftLead.getSelectedSensorPosition(), rightLead.getSelectedSensorPosition());

  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftLead.getSelectedSensorVelocity(), rightLead.getSelectedSensorVelocity());
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLead.setVoltage(leftVolts);
    rightLead.setVoltage(-rightVolts);
    m_drive.feed();
  }
  public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
    double leftSpeed = speed - rotation * dir;//(quickTurn ? -rotation : speed - (speed != 0 ? rotation : 0));
    double rightSpeed = speed + rotation *dir;//(quickTurn ? rotation : speed + (speed != 0 ? rotation : 0));
    leftLead.set(ControlMode.PercentOutput, leftSpeed);
    rightLead.set(ControlMode.PercentOutput, rightSpeed);
    leftFollow.set(ControlMode.PercentOutput, leftSpeed);
    rightFollow.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void tankDrive (double leftSpeed, double rightSpeed) {
    if (shootFront){
    leftLead.set(ControlMode.PercentOutput, leftSpeed);
    rightLead.set(ControlMode.PercentOutput, rightSpeed);
    leftFollow.set(ControlMode.PercentOutput, leftSpeed);
    rightFollow.set(ControlMode.PercentOutput, rightSpeed);
    } else{
      leftLead.set(ControlMode.PercentOutput, rightSpeed);
      rightLead.set(ControlMode.PercentOutput, leftSpeed);
      leftFollow.set(ControlMode.PercentOutput, rightSpeed);
      rightFollow.set(ControlMode.PercentOutput, leftSpeed);
    }
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
  leftFollow.set(ControlMode.PercentOutput, 0);
  rightLead.set(ControlMode.PercentOutput, 0); 
  rightFollow.set(ControlMode.PercentOutput, 0);
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
    leftFollow.follow(rightLead, FollowerType.AuxOutput1);
  }

  public boolean onTarget() {
    return Math.abs(leftLead.getClosedLoopError()) < 1000;
  }

  public double getError() {
    return leftLead.getClosedLoopError();
  }

  public void resetEncoder(){
    leftLead.getSensorCollection().setIntegratedSensorPosition(0, 30); 
    rightLead.getSensorCollection().setIntegratedSensorPosition(0, 30); 
  }
  
  public void resetOdometry(Pose2d pose) {
    resetEncoder();
    m_odometry.resetPosition(pose, new Rotation2d(getAngle()));
  }
  
  public void zeroAllSensors() {
    leftLead.getSensorCollection().setIntegratedSensorPosition(0, 30); 
    rightLead.getSensorCollection().setIntegratedSensorPosition(0, 30); 
    pidgey.setYaw(0, 30);
    pidgey.setAccumZAngle(0, 30);
  }
	
  public double getAngle() {
    double[] ypr = new double[3];
    pidgey.getYawPitchRoll(ypr);
    return ypr[0];
    //return rightLead.getSelectedSensorPosition(1);
  }
	
  public void switchDirection(){
    dir *= -1;
    shootFront = !shootFront;
    leftLead.setInverted(!leftLead.getInverted());
    leftFollow.setInverted(!leftFollow.getInverted());
    rightLead.setInverted(!rightLead.getInverted());
    rightFollow.setInverted(!rightFollow.getInverted());
    
  }
  public void stop() {
    leftLead.set(ControlMode.PercentOutput, 0); 
    rightLead.set(ControlMode.PercentOutput, 0); 
  }
	
  public ArrayList<WPI_TalonFX> getInstruments() {
	return _instruments;	  
  }
  public String camera2Path(){
    
    String poof = "erebet";
    return poof;
  }
}
// " hey buddy if you could just switch these motors to the other dirction that'd be great"

/* if(programmers == tired){
  println("NAP"); 
 }else {
   println("Keep programming"); 
 }; */
