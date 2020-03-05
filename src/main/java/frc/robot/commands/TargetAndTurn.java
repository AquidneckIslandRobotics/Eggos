/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TargetAndTurn extends SequentialCommandGroup {
  /**
   * Creates a new EightCellAutoPart3.
   */
  public TargetAndTurn(Chassis chassis, Turret turret) {
   super(new TargetWhileDriving(chassis, turret), new TurnPID(chassis, 45));
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
  }
}
