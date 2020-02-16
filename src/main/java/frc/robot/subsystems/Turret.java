/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Turret extends SubsystemBase {
    public static TalonSRX turretRotate = new TalonSRX(Constants.TurretRotate);
    public static WPI_TalonFX turretWheel1 = new WPI_TalonFX(Constants.LeftShooter);
    public static WPI_TalonFX turretWheel2 = new WPI_TalonFX(Constants.RightShooter);
    //public static Encoder turretEncoder = new Encoder(Constants.TurretEncoder); 
    public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    public static Turret m_turret = new Turret();
  /**
   * Creates a new Turret.
   */
  public Turret() {
    turretRotate.setSensorPhase(false);
    turretRotate.setInverted(false);
    turretRotate.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    
   // m_analogSensor = turretServo.;

    //PID Coefficients 
   /* kP = 0.9; 
    kI = 0.1;
    kD = 0.1;
    kIz = 0.1; 
    kFF = 0.1; 
    kMaxOutput = 1; 
    kMinOutput = -1; */ 

   /* m_pidController.setP(kP); 
    m_pidController.setI(kI); 
    m_pidController.setD(kD); 
    m_pidController.setIZone(kIz); 
    m_pidController.setFF(kFF); 
    m_pidController.setOutputRange(kMinOutput, kMaxOutput); */ 

    SmartDashboard.putNumber("Turret Rotations", 0); 

    //turretServo.setNeutralMode(NeutralMode.Brake);
   // turretServo.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
   //this won't work bc turret servo is a Neo 550
  }

  public double getLimelightX() {
    return tx.getDouble(0);
  }

  public void lightsOn() {
    table.getEntry("ledMode").setNumber(3);
  }


  public void lightsOff() {
    table.getEntry("ledMode").setNumber(1);
  }
  
 // public double getTurretEncoder() {
  //  return turretEncoder.get(); 
 // }

  //public void resetEncoders() {
   // turretEncoder.reset(); 
 // }
 
 /* public void resetTurret() {
  } */ 

  @Override
  public void periodic() {

    double rotations = SmartDashboard.getNumber("Turret Rotations", 0); 
   // m_pidController.setReference(rotations, ControlType.kPosition); 
    SmartDashboard.putNumber("Turret Set Point", rotations); 
   // SmartDashboard.putNumber("Process Variable Conversion", m_analogSensor.getPositionConversionFactor()); 
   //SmartDashboard.putNumber("Process Variable Get Pos only", m_analogSensor()); 
    SmartDashboard.putNumber("Sensor Velocity", turretRotate.getSelectedSensorVelocity()); 
    SmartDashboard.putNumber("Sensor Position", turretRotate.getSelectedSensorPosition()); 
    
    
   


    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
   // SmartDashboard.putNumber("Turret Encoder", turretEncoder); 
    // This method will be called once per scheduler run
  }
    public void setSpeed(double speed) {
    turretRotate.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("Turret Speed", speed);
    //turretServo.getFaults(_faults); 
  
  }
    public void stopTurret(){
    turretRotate.set(ControlMode.PercentOutput, 0);
    SmartDashboard.putNumber("Turret Speed", 0);
  }
    public void aim(){
      double xValue = getLimelightX();
      double speed = xValue * -0.035;
      setSpeed(speed);
    }

  public void startWheel() {
  turretWheel1.set(ControlMode.PercentOutput, -0.85);
  turretWheel2.set(ControlMode.PercentOutput, 0.85);
}
  public void stopWheel() {
  turretWheel1.set(ControlMode.PercentOutput, 0);
  turretWheel2.set(ControlMode.PercentOutput, 0);
}


  
    // This method will be called once per scheduler run
  }

