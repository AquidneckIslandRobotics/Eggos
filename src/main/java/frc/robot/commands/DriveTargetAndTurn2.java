/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveTargetAndTurn2 extends ParallelCommandGroup {
  /**
   * Creates a new TargetIntakeAndTurn.
   */
  public DriveTargetAndTurn2(Chassis chassis, Intake takeInCells, Shooter shooter, Turret turret) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(new SetLocate(shooter, turret, 3), new Hood2Auto(turret, Constants.hoodLocate[turret.hoodLocate]),new TurretLimelightAuto(turret), new MotionMagicWithoutIntake(chassis, -180));
  }
}
