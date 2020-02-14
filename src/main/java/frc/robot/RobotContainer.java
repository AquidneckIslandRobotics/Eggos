/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoColor;
import frc.robot.commands.ColorSpinFour;
import frc.robot.commands.ColorSpinTarget;
import frc.robot.commands.Drive;
import frc.robot.commands.MotionMagic;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ColorSensor; 
import edu.wpi.first.wpilibj.GenericHID.Hand; 

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chassis m_chassis = new Chassis();
  
  private Button button = new JoystickButton(drivingJoystick1, 6);
  
private static XboxController drivingJoystick1 = new XboxController(1);
public static XboxController colorJoystick = new XboxController(2); 

  private final MotionMagic c_MotionMagic = new MotionMagic(m_chassis, 10);
  public final ColorSensor m_colorSensor = new ColorSensor(); 

  public static double getRightX() {
    double rightX = colorJoystick.getX(Hand.kRight); 
    if(Math.abs(rightX) < 0.05) {
      rightX = 0; 
    }
    return rightX; 
  }

  Button colorA = new JoystickButton(colorJoystick, 1); 
  Button colorB = new JoystickButton(colorJoystick, 2); 
  Button colorX = new JoystickButton(colorJoystick, 3); 
  Button colorY = new JoystickButton(colorJoystick, 4); 
  Button colorLB = new JoystickButton(colorJoystick, 5); 
  Button colorRB = new JoystickButton(colorJoystick, 6); 

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
    m_chassis.setDefaultCommand(new Drive(m_chassis, drivingJoystick1, button));

    //Color Controller Buttons 
    colorA.whenPressed(new AutoColor("Green")); 
    colorB.whenPressed(new AutoColor("Red")); 
    colorX.whenPressed(new AutoColor("Blue")); 
    colorY.whenPressed(new AutoColor("Yellow")); 
    colorLB.whileHeld(new ColorSpinTarget()); 
    colorRB.whenPressed(new ColorSpinFour()); 
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
