/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShootToggle extends CommandBase {
  private Shooter m_shooter;
  private Turret m_turret;
  private boolean m_forward;
  /**
   * Creates a new ShootToggle.
   */
  public ShootToggle(Shooter shooter, Turret turret, boolean forward) {
    m_shooter = shooter;
    m_turret = turret;
    m_forward = forward;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_forward){
      if (m_shooter.shootLocate >= 4) m_shooter.shootLocate = 4;
      else m_shooter.shootLocate++;
      
      if (m_turret.hoodLocate >= 4) m_turret.hoodLocate = 4;
      else m_turret.hoodLocate++;
    }else{
      if(m_shooter.shootLocate <=0) m_shooter.shootLocate = 0;
      else m_shooter.shootLocate--;
      
      if (m_turret.hoodLocate <= 0) m_turret.hoodLocate = 0;
      else m_turret.hoodLocate--;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
