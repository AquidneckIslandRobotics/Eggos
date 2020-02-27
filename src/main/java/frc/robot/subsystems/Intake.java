/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax rightIntake = new CANSparkMax(Constants.RightIntake, MotorType.kBrushless);
  CANSparkMax leftIntake = new CANSparkMax(Constants.LeftIntake, MotorType.kBrushless);
  DoubleSolenoid solenoid = new DoubleSolenoid(0,1);
  /**
   * Creates a new Intake.
   */
  public Intake() {
    solenoid.set(Value.kReverse);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void IntakeInward() {
    rightIntake.set(0.85);
    leftIntake.set(-0.85);
    solenoid.set(Value.kForward);
  }

  public void DeployIntake() {
    solenoid.set(Value.kForward);
  }

  public void StopIntake() {
    rightIntake.set(0);
    leftIntake.set(0);
    solenoid.set(Value.kReverse);
  }
}
// Just trying to get this to commit
