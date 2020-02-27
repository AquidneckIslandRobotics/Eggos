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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Turret extends SubsystemBase {
    private static CANSparkMax turretRotate = new CANSparkMax(Constants.TurretRotate, MotorType.kBrushless);
    //public static Encoder turretEncoder = new Encoder(Constants.TurretEncoder); 
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private static NetworkTableEntry tx = table.getEntry("tx");
    private static NetworkTableEntry ty = table.getEntry("ty");
    private static NetworkTableEntry ta = table.getEntry("ta");
    //public double
    WPI_TalonFX hood = new WPI_TalonFX(Constants.HoodAngle);
    //public static Turret m_turret = new Turret();
  /**
   * Creates a new Turret.
   */
  public Turret() {
    turretRotate.setInverted(false);

    turretRotate.setIdleMode(IdleMode.kBrake);
    
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
    //SmartDashboard.putNumber("Sensor Velocity", turretRotate.getSelectedSensorVelocity()); 
    //SmartDashboard.putNumber("Sensor Position", turretRotate.getSelectedSensorPosition()); 
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
    turretRotate.set(speed);
    SmartDashboard.putNumber("Turret Speed", speed);
    //turretServo.getFaults(_faults); 
  
  }
    public void stopTurret(){
    turretRotate.set(0);
    SmartDashboard.putNumber("Turret Speed", 0);
  }
    public void aim(){
      double xValue = getLimelightX();
      double speed = xValue * -0.035;
      setSpeed(speed);
    }

    public double getHoodAngle() {
      hood.getSelectedSensorPosition();
      return hood.getSelectedSensorPosition();
    }

    public void setHoodAngle(double hoodAngle) {
      hood.set(ControlMode.PercentOutput, hoodAngle);
    }
    public void resetEncoder(){ 
      hood.getSensorCollection().setIntegratedSensorPosition(0, 0); 
     }
    // This method will be called once per scheduler run
  }

