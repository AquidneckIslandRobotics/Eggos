/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ThreeCellAuto2WithoutIntake extends SequentialCommandGroup {
  /**
   * Creates a new ThreeCellAuto.
   */
  public ThreeCellAuto2WithoutIntake(Chassis chassis, Intake takeInCells, Shooter shooter, Turret turret) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    // super(new ShooterAuto(shooter), new IntakeInward(takeInCells), new EightCellAutoWithoutShooter(chassis, takeInCells, shooter, turret));
    super(new SetLocate(shooter, turret, 1), new Hood2Auto(turret, Constants.hoodLocate[turret.hoodLocate]), new ShooterAutoTimed(shooter), new MotionMagicWithoutIntake(chassis, -80));
  }
}
