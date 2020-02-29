/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.music.Orchestra;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.HopperIntake;

public class Shooter extends SubsystemBase {

  
  private static TalonFX shooterWheel1 = new TalonFX(Constants.LeftShooter);
  private static TalonFX shooterWheel2 = new TalonFX(Constants.RightShooter);
  private static CANSparkMax hopperRight = new CANSparkMax(Constants.HopperRight, MotorType.kBrushless);
  private static CANSparkMax hopperLeft = new CANSparkMax(Constants.HopperLeft, MotorType.kBrushless);
  private static CANSparkMax feed = new CANSparkMax(Constants.Feed, MotorType.kBrushless);

  public int shootLocate = 0;

  private double feedGain = 1;

  private static TalonFXConfiguration _velocity_closed = new TalonFXConfiguration();

  private int hopperCount;
  private boolean hopperDir = true; // true = forwards, false = reverse

    private ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    // Reset motor config
    shooterWheel1.configFactoryDefault();
    shooterWheel2.configFactoryDefault();
    // Have second motor follow first
    shooterWheel2.follow(shooterWheel1);
    shooterWheel2.configNeutralDeadband(0); // This is needed to not overhead leader

    // Set correct motor direction and sensor orientation
    shooterWheel1.setInverted(TalonFXInvertType.Clockwise);
    shooterWheel1.setSensorPhase(false);
    shooterWheel2.setInverted(TalonFXInvertType.CounterClockwise);

    // Make sure motors are in coast mode
    shooterWheel1.setNeutralMode(NeutralMode.Coast);
    shooterWheel2.setNeutralMode(NeutralMode.Coast);

    // Config all PID settings
    _velocity_closed.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    _velocity_closed.nominalOutputForward = 0;
    _velocity_closed.nominalOutputReverse = 0;
    _velocity_closed.peakOutputForward = 1;
    _velocity_closed.peakOutputReverse = 0; // Should never go in reverse
    _velocity_closed.slot0.kF = 0.04843;//0.05;
    _velocity_closed.slot0.kP = 0.078;//0.02;
    _velocity_closed.slot0.kI = 0;
    _velocity_closed.slot0.kD = 0;
    

    shooterWheel1.configAllSettings(_velocity_closed);

    shooterWheel1.selectProfileSlot(0,0);

    // Music
    _instruments.add(shooterWheel1);
    _instruments.add(shooterWheel2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("L Hopper", hopperLeft.get());
    SmartDashboard.putNumber("R Hopper", hopperRight.get());
    SmartDashboard.putNumber("Turret Wheel Velocity", shooterWheel1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("output Current", feed.getOutputCurrent());
    SmartDashboard.putNumber("shootLocate", shootLocate+1);
    feedGain = SmartDashboard.getNumber("Feed Gain",feedGain);
  }

  /**
   * Runs hopper but takes amperage into account
   */
  public void autoHopper() {
    if (hopperDir) {
      HopperIntake();
      if (feed.getOutputCurrent() > 15) {
        if (hopperCount++ > 18) {
          hopperDir = false;
          hopperCount = 0;
        }
      }
    } else {
      if (hopperCount++ < 70)
        HopperOuttake();
      else {
        hopperDir = true;
        hopperCount = 0;
      }
    }
  }

  public void HopperIntake() {
    hopperRight.set(-0.3 * Constants.feedGain[shootLocate]);
    hopperLeft.set(-0.3 * Constants.feedGain[shootLocate]);
    feed.set(1 * Constants.feedGain[shootLocate]);
  }

  public void HopperOuttake() {
    hopperRight.set(-0.3);
    hopperLeft.set(0.3);
    feed.set(-1);
  }

  public void stopHopper(){
    hopperRight.set(0);
    hopperLeft.set(0);
    feed.set(0);
    hopperDir = true;
    hopperCount = 0;
  }
  /**
   * Method to set the velocity control of shooter wheels
   * @param velocity The speed of the wheels in RPMs
   */
  public void startWheel(double velocity){
    velocity = ((velocity * 2048) / 600); // Convert velocity in RPM to Units Per 100ms
    shooterWheel1.set(ControlMode.Velocity, velocity);
  }

  public boolean getVelocityOnTarget() {
    double err = shooterWheel1.getClosedLoopError(0);
    return (Math.abs(err) <= Constants.kVelocityDeadband);
  }

  /**
   * Method to start shooter wheels at a percent output
   */
  public void startWheel() {
    shooterWheel1.set(ControlMode.PercentOutput, -0.85);
    shooterWheel2.set(ControlMode.PercentOutput, 0.85);
  }

  /**
   * Method to stop the shooter wheels
   */
  public void stopWheel() {
    shooterWheel1.set(ControlMode.PercentOutput, 0);
    shooterWheel2.set(ControlMode.PercentOutput, 0);
  }
  
  public ArrayList<TalonFX> getInstruments() {
    return _instruments;
  }
}
