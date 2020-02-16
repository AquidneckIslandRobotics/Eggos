/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveDistanceAuto;
import frc.robot.commands.MotionMagic;
import frc.robot.commands.switchDirection;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Turret;
import frc.robot.commands.TurretTurn;
import frc.robot.commands.TurretTarget;
import frc.robot.commands.SpinWheel;
import frc.robot.commands.TurretLimelight;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // Subsystems
  
  private final Turret m_turret = new Turret();
  
  // Joysticks
  private static XboxController manipulatorJoystick = new XboxController(0);
  private static XboxController drivingJoystick1 = new XboxController(1);
  
  // Buttons
  private Button button = new JoystickButton(drivingJoystick1, 6);
  
  private Button MotionMagicButton = new JoystickButton(drivingJoystick1, 3); //button x 
  private Button RT = new JoystickButton(manipulatorJoystick, 7);
  private Button limeTime = new JoystickButton(manipulatorJoystick, 4);

  private Button driverA = new JoystickButton(manipulatorJoystick, 1); 
  private Button manipulatorB = new JoystickButton(manipulatorJoystick, 2);
  private Button manipulatorX = new JoystickButton(manipulatorJoystick, 3);

  
  private Button driverYeet = new JoystickButton(drivingJoystick1, 4);
  private Button flipDirectionButton = new JoystickButton(drivingJoystick1, 5); 
  private Button AutoAButton = new JoystickButton(drivingJoystick1, 1); 
  
  // Commands
  private final MotionMagic c_MotionMagic = new MotionMagic(Robot.m_chassis, 10);

  // ------------------------------------------

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }
  
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default commands
    Robot.m_chassis.setDefaultCommand(new Drive(Robot.m_chassis, drivingJoystick1, button, driverYeet));
    
    // Button Setup
    //  Driver Buttons
    driverA.whileHeld(new TurretTarget(m_turret));
    flipDirectionButton.whenPressed(new switchDirection(Robot.m_chassis));

    // Manipulator Buttons
    manipulatorB.whileHeld(new TurretTurn(m_turret, .5));
    manipulatorX.whileHeld(new TurretTurn(m_turret, -.5));
    RT.whileHeld(new SpinWheel(m_turret));
    limeTime.whileHeld(new TurretLimelight(m_turret));
    flipDirectionButton.whenPressed(new switchDirection(Robot.m_chassis)); 
    AutoAButton.whenPressed(new DriveDistanceAuto(Robot.m_chassis, 12));
    MotionMagicButton.whenPressed(new MotionMagic(Robot.m_chassis, 12)); 
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return c_MotionMagic;
  }
}
