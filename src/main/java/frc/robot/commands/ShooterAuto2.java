/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;


public class ShooterAuto2 extends CommandBase {
 private Shooter shooter;
 public double startTime; 
 //private Turret turret;

  /**
   * Creates a new ShooterAuto.
   */
  public ShooterAuto2(Shooter shooter) {
    this.shooter = shooter;
   // this.turret = turret;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp(); 
    //start at rotation 0
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  //  turret.aim();
    shooter.startWheel(5000);
   if( Timer.getFPGATimestamp() > startTime + 1.25)
       shooter.autoHopper();

    //shoot 3 balls at beginning(initiation line)


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopHopper();
    shooter.stopWheel();
   // turret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(Timer.getFPGATimestamp() > startTime + 5); 
    
  }
}
