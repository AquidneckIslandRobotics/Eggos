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
import frc.robot.commands.DriveDistanceAuto;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveAndSpinGroup extends SequentialCommandGroup {
public final Chassis m_chassis = new Chassis(); // is this being static okay
  public final Turret m_turret = new Turret();
  public Chassis m_drive; 
  /**
   * Creates a new DriveAndSpinGroup.
   */
  public DriveAndSpinGroup() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    //super(new DriveDistanceAuto(m_drive, 100), new SpinWheel(m_turret));// SpinWheel may be wrong command
    //100 is probably too much
  }
}