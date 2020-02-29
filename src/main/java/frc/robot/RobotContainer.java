/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Climb;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.Drive;
//import frc.robot.commands.DriveAndSpinGroup;
import frc.robot.commands.DriveDistanceAuto;
import frc.robot.commands.Hood;
import frc.robot.commands.Hood2;
import frc.robot.commands.HoodAndLime;
import frc.robot.commands.HopperIntake;
import frc.robot.commands.HopperOuttake;
import frc.robot.commands.MotionMagic;
//import frc.robot.commands.Music;
import frc.robot.commands.AutoShootVelocity;
import frc.robot.commands.ShootAndDrive;
import frc.robot.commands.ShootToggle;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeInward;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.commands.TurretTurn;
import frc.robot.commands.TurretTarget;
import frc.robot.commands.SpinWheel;
import frc.robot.commands.TurretLimelight;
import frc.robot.commands.TurretPID;

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
  public  XboxController manipulatorJoystick = new XboxController(0);
  private static XboxController drivingJoystick1 = new XboxController(1);
  private static XboxController extraJoystick = new XboxController(3);
  
  // Buttons
  private Button driverX = new JoystickButton(drivingJoystick1, 3);
  private Button driverYeet = new JoystickButton(drivingJoystick1, 4);
  private Button driverLB = new JoystickButton(drivingJoystick1, 5);
  private Button driverRB = new JoystickButton(drivingJoystick1, 6);
  private Button driverBack = new JoystickButton(drivingJoystick1, 7);
  private Button driverStart = new JoystickButton(drivingJoystick1, 8);
  private Button driverA = new JoystickButton(drivingJoystick1, 1);
  private Button driverB = new JoystickButton(drivingJoystick1, 2);
  //private Button MotionMagicButton = new JoystickButton(drivingJoystick1, 3); //button x 
  //private Button RT = new JoystickButton(manipulatorJoystick, 7);
  //private Button limeTime = new JoystickButton(manipulatorJoystick, 4);

  //private Button driverA = new JoystickButton(manipulatorJoystick, 1); 
  //private Button manipulatorB = new JoystickButton(manipulatorJoystick, 2);
  //private Button manipulatorX = new JoystickButton(manipulatorJoystick, 3);

  private Button manipulatorA = new JoystickButton(manipulatorJoystick, 1); 
  private Button manipulatorB = new JoystickButton(manipulatorJoystick, 2);
  private Button manipulatorX = new JoystickButton(manipulatorJoystick, 3);
  private Button manipulatorY = new JoystickButton(manipulatorJoystick, 4);
  private Button manipulatorLimeLB = new JoystickButton(manipulatorJoystick, 5);
  private Button manipulatorRB = new JoystickButton(manipulatorJoystick, 6);
  private Button manipulatorL3 = new JoystickButton(manipulatorJoystick, 9);//Joystick press
  private Button manipulatorR3 = new JoystickButton(manipulatorJoystick, 10);//Joystick press

  // Triggers for inputs that are not buttons
  public Trigger climbTriggerL = new Trigger((BooleanSupplier)() -> drivingJoystick1.getTriggerAxis(Hand.kLeft) > .5);
  public Trigger climbTriggerR = new Trigger((BooleanSupplier)() -> drivingJoystick1.getTriggerAxis(Hand.kRight) > .5);
  public Trigger advancePosition = new Trigger((BooleanSupplier)() -> manipulatorJoystick.getPOV() == 0);
  public Trigger reteatPosition = new Trigger((BooleanSupplier)() -> manipulatorJoystick.getPOV() == 180);

  public Button extraLB = new JoystickButton(extraJoystick, 5);
  public Button extraRB = new JoystickButton(extraJoystick, 6);

  // Commands
  private final MotionMagic c_MotionMagic = new MotionMagic(m_chassis, -180, m_intake);
  private final SixCellAuto m_sixCellAuto = new SixCellAuto(m_chassis, m_intake, m_shooter, m_turret);

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
    m_chassis.setDefaultCommand(new Drive(m_chassis, drivingJoystick1, driverRB, driverYeet));
    //m_shooter.setDefaultCommand(new Music(m_shooter, ""));
    
    // Button Setup
    //  Driver Buttons
   // driverA.whenPressed(new Hood(0, m_turret));
   // driverB.whenPressed(new Hood(100, m_turret));
    driverLB.whenPressed(new switchDirection(m_chassis));
    //driverBack.whileHeld(new Climb(m_climber, 0.5));
    //driverStart.whileHeld(new Climb(m_climber, 0.75));

    climbTriggerL.and(climbTriggerR).whileActiveOnce(new Climb(m_climber, 0.75));

    // Manipulator Buttons
    manipulatorA.whileHeld(new HopperOuttake(m_shooter));
    manipulatorB.whileHeld(new HopperIntake(m_shooter));
    manipulatorX.whileHeld(new IntakeInward(m_intake));
    manipulatorY.whileHeld(new DeployIntake(m_intake));
    manipulatorLimeLB.whileHeld(new HoodAndLime(m_turret));//TurretLimelight(m_turret));
    manipulatorRB.whileHeld(new SpinWheel(m_shooter));//AutoShootVelocity(m_shooter, m_turret, 5000));//
    manipulatorL3.whileHeld(new TurretTurn(m_turret, .5));
    manipulatorR3.whileHeld(new TurretTurn(m_turret, -.5));
    advancePosition.whenActive(new ShootToggle(m_shooter, m_turret, false));
    reteatPosition.whenActive(new ShootToggle(m_shooter, m_turret, true));

    SmartDashboard.putData("Hood 1", new Hood2(m_turret, Constants.hoodLocate[0]));
    SmartDashboard.putData("Hood 4",new Hood2(m_turret, Constants.hoodLocate[3]));

    // SmartDashboard Buttons
    SmartDashboard.putData(new UnClimb(m_climber));
    SmartDashboard.putData(new ShootToggle(m_shooter, m_turret, false));
    SmartDashboard.putData("shootRetreat", new ShootToggle(m_shooter, m_turret, true));

    //Triggers
    //boolean test = drivingJoystick1.getTriggerAxis(Hand.kLeft) > .5;
    //climbTrigger.;

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return c_MotionMagic;//m_sixCellAuto;
  }
}
