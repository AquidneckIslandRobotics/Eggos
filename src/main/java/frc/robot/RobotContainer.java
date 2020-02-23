/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveAndSpinGroup;
import frc.robot.commands.DriveDistanceAuto;
import frc.robot.commands.HopperIntake;
import frc.robot.commands.HopperOuttake;
import frc.robot.commands.MotionMagic;
import frc.robot.commands.AutoShootVelocity;
import frc.robot.commands.Music;
import frc.robot.commands.ShooterAuto;
import frc.robot.commands.SixCellAuto;
import frc.robot.commands.UnClimb;
import frc.robot.commands.switchDirection;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeInward;
import frc.robot.subsystems.Intake;
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
  private final static Chassis m_chassis = new Chassis();
  private final static Climber m_climber = new Climber();
  private final static Intake m_intake = new Intake();
  private final static Shooter m_shooter = new Shooter();
  private final static Turret m_turret = new Turret();
  
  // Joysticks
  private static XboxController manipulatorJoystick = new XboxController(0);
  private static XboxController drivingJoystick1 = new XboxController(1);
  private static XboxController extraJoystick = new XboxController(3); 
  
  // Buttons
  private Button driverX = new JoystickButton(drivingJoystick1, 3);
  private Button driverYeet = new JoystickButton(drivingJoystick1, 4);
  private Button driverLB = new JoystickButton(drivingJoystick1, 5);
  private Button driverRB = new JoystickButton(drivingJoystick1, 6);
  private Button driverBack = new JoystickButton(drivingJoystick1, 7);
  private Button driverStart = new JoystickButton(drivingJoystick1, 8);

  private Button manipulatorA = new JoystickButton(manipulatorJoystick, 1); 
  private Button manipulatorB = new JoystickButton(manipulatorJoystick, 2);
  private Button manipulatorX = new JoystickButton(manipulatorJoystick, 3);
  private Button manipulatorY = new JoystickButton(manipulatorJoystick, 4);
  private Button manipulatorLimeLB = new JoystickButton(manipulatorJoystick, 5);
  private Button manipulatorRB = new JoystickButton(manipulatorJoystick, 6);
  private Button manipulatorL3 = new JoystickButton(manipulatorJoystick, 9);//Joystick press
  private Button manipulatorR3 = new JoystickButton(manipulatorJoystick, 10);//Joystick press

  private Button extraButtonA = new JoystickButton(extraJoystick, 1); 
  private Button extraButtonB = new JoystickButton(extraJoystick, 2); 
  private Button extraButtonX = new JoystickButton(extraJoystick, 3); 
  private Button extraButtonY = new JoystickButton(extraJoystick, 4);
  private Button extraButtonLB = new JoystickButton(extraJoystick, 5); 
  private Button extraButtonRB = new JoystickButton(extraJoystick, 6); 
  private Button extraStart = new JoystickButton(extraJoystick, 8); 


  // Commands
  private final MotionMagic c_MotionMagic = new MotionMagic(m_chassis, 10);
  private final Music c_Music = new Music(m_shooter, "");

  // ------------------------------------------

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    SmartDashboard.putBoolean("Prime Climb Reset", m_climber.unclimb);
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
    
    // Button Setup
    //  Driver Buttons
    //driverA.whileHeld(new TurretTarget(m_turret));
    //flipDirectionButton.whenPressed(new switchDirection(Robot.m_chassis));
    m_chassis.setDefaultCommand(new Drive(m_chassis, drivingJoystick1, driverRB, driverYeet));
    //m_shooter.setDefaultCommand(new Music(m_shooter, ""));
    
    // Button Setup
    //  Driver Buttons
    driverLB.whenPressed(new switchDirection(m_chassis));
    //driverX.whileHeld(new);
    driverRB.whileHeld(new IntakeInward(m_intake));
    driverBack.whileHeld(new Climb(m_climber, 0.5));
    driverStart.whileHeld(new Climb(m_climber, 0.75));

    // Manipulator Buttons
    manipulatorA.whileHeld(new TurretTarget(m_turret));
    manipulatorB.whileHeld(new TurretTurn(m_turret, .5));
    //manipulatorX.whileHeld(new TurretTurn(m_turret, -.5));
    manipulatorX.whileHeld(new HopperIntake(m_shooter));
    manipulatorY.whileHeld(new HopperOuttake(m_shooter));
    manipulatorLimeLB.whileHeld(new TurretLimelight(m_turret));
    manipulatorRB.whileHeld(new SpinWheel(m_shooter));//AutoShootVelocity(m_shooter, m_turret, 5000));//
    manipulatorR3.whileHeld(new TurretTurn(m_turret, .5));
    manipulatorL3.whileHeld(new TurretTurn(m_turret, -0.5));

    extraButtonA.whenPressed(new SixCellAuto(m_chassis, m_intake, m_shooter, m_turret));
   // extraButtonA.whenPressed(new DriveDistanceAuto(m_chassis, 100)); 
    extraButtonB.whenPressed(new DriveDistanceAuto(m_chassis, -100)); 
    extraButtonX.whenPressed(new DriveDistanceAuto(m_chassis, 12)); 
    extraButtonY.whenPressed(new DriveDistanceAuto(m_chassis, -12)); 
    extraButtonRB.whenPressed(new MotionMagic(m_chassis, 100)); 
    extraButtonLB.whenPressed(new DriveAndSpinGroup(m_chassis, m_intake)); 
    extraStart.whenPressed(new ShooterAuto(m_shooter)); 
    // SmartDashboard Buttons
    SmartDashboard.putData(new UnClimb(m_climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return c_Music;
  }
}
