/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Chassis;

public class Music extends CommandBase {
  private static Shooter m_shooter;
  private static Chassis m_chassis;
  private static String m_name;
  private Orchestra _orchestra;
  private ArrayList<TalonFX> _instruments;
  /**
   * Creates a new Music.
   */
  public Music(Chassis chassis, Shooter shooter, String name) {
    m_shooter = shooter;
    m_chassis = chassis;
    m_name = name;
    addRequirements(chassis, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _instruments = m_chassis.getInstruments();
    _instruments.addAll(m_shooter.getInstruments());
    _orchestra = new Orchestra(_instruments);
    _orchestra.loadMusic(m_name);
    _orchestra.play();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _orchestra.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !(_orchestra.isPlaying());
  }
}
