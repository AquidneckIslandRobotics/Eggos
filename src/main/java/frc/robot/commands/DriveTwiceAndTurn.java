/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveTwiceAndTurn extends SequentialCommandGroup {
  /**
   * Creates a new DriveTwiceAndTurn.
   */
  public DriveTwiceAndTurn(Chassis chassis, Shooter shooter) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new MotionMagicWithoutIntake(chassis, -180), new MotionMagicWithoutIntake(chassis, 112)); //new WaitCommand(1), new TurnPID(chassis, shooter, -50), new MotionMagicWithoutIntake(chassis, -50));
  }
}
