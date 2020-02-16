/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  private TalonFX leftLead;
  private BaseMotorController leftFollow;
  private TalonFX rightLead;
  private BaseMotorController rightFollow;

  // public Encoder rightEncoder = new Encoder(Constants.EncoderRA, Constants.EncoderRB);
  // public Encoder leftEncoder = new Encoder(Constants.EncoderLA, Constants.EncoderLB);
  public TalonFXConfiguration _motion_magic = new TalonFXConfiguration();

  public TalonFXConfiguration _leftConfig = new TalonFXConfiguration(); 
  public TalonFXConfiguration _rightConfig = new TalonFXConfiguration(); 

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

   // setRobotConfigs(_rightInvert, _rightConfig); 
    _rightConfig.slot0.kF = Constants.kGains_Distanc.kF; 
    _rightConfig.slot0.kP = Constants.kGains_Distanc.kP;
		_rightConfig.slot0.kI = Constants.kGains_Distanc.kI;
		_rightConfig.slot0.kD = Constants.kGains_Distanc.kD;
		_rightConfig.slot0.integralZone = Constants.kGains_Distanc.kIzone;
    _rightConfig.slot0.closedLoopPeakOutput = Constants.kGains_Distanc.kPeakOutput;
   // _rightConfig.remoteFilter1.remoteSensorDeviceID = _pidgey.getDeviceID();    //Pigeon Device ID
	///	_rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw; //This is for a Pigeon over CAN
	//	_rightConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1; //Set as the Aux Sensor
    //_rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 3600.0 / Constants.kPigeonUnitsPerRotation;
    
    _leftConfig.neutralDeadband = Constants.kNeutralDeadband;
    _rightConfig.neutralDeadband = Constants.kNeutralDeadband;

    _rightConfig.motionAcceleration = 2000; //(distance units per 100 ms) per second
    _rightConfig.motionCruiseVelocity = 2000; //distance units per 100 ms
    
    _leftConfig.motionAcceleration = 2000; //(distance units per 100 ms) per second
		_leftConfig.motionCruiseVelocity = 2000; //distance units per 100 ms

    leftLead.configAllSettings(_leftConfig);
		rightLead.configAllSettings(_rightConfig); 
    
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Lead Direction", leftLead.getInverted());
    SmartDashboard.putBoolean("Right Lead Direction", rightLead.getInverted());

    SmartDashboard.putNumber("Left Encoder Count", leftLead.getSelectedSensorPosition()); 
    SmartDashboard.putNumber("Right Encoder COUNT", rightLead.getSelectedSensorPosition()); 

    SmartDashboard.putNumber("ENCODER Encoder Count", leftLead.getSelectedSensorVelocity()); 
    // This method will be called once per scheduler run


  }

  public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
    double leftSpeed = (quickTurn ? -rotation : speed - (speed != 0 ? rotation : 0));
    double rightSpeed = (quickTurn ? rotation : speed + (speed != 0 ? rotation : 0));

    leftLead.set(ControlMode.PercentOutput, leftSpeed);
    rightLead.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void setConfig(TalonFXConfiguration config) {
    leftLead.configAllSettings(config);
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
    rightLead.set(ControlMode.MotionMagic, setpoint); 
  }

  public void resetEncoder(){
  leftLead.getSensorCollection().setIntegratedSensorPosition(0, 30); 
  rightLead.getSensorCollection().setIntegratedSensorPosition(0, 30); 
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
}
// " hey buddy if you could just switch these motors to the other dirction that'd be great"

/* if(programmers == tired){
  println("NAP"); 
 }else {
   println("Keep programming"); 
 }; */