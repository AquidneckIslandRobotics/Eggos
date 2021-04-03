/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.Climb;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.Drive;
//import frc.robot.commands.DriveAndSpinGroup;
import frc.robot.commands.DriveDistanceAuto;
import frc.robot.commands.EightCellAuto;
import frc.robot.commands.Hood;
import frc.robot.commands.Hood2;
import frc.robot.commands.HoodAndLime;
import frc.robot.commands.HopperIntake;
import frc.robot.commands.HopperOuttake;
import frc.robot.commands.IntakeAndHopper;
import frc.robot.commands.MotionMagic;
//import frc.robot.commands.Music;
//import frc.robot.commands.Music;
import frc.robot.commands.AutoShootVelocity;
import frc.robot.commands.ShootAndDrive;
import frc.robot.commands.ShootToggle;
import frc.robot.commands.ShooterAuto;
import frc.robot.commands.ShooterAutoTimed;
import frc.robot.commands.SixCellAuto;
import frc.robot.commands.SixCellAuto2;
import frc.robot.commands.UnClimb;
import frc.robot.commands.switchDirection;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeInward;
import frc.robot.commands.IntakeOutward;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.commands.TurretTurn;
import frc.robot.commands.TurretTarget;
import frc.robot.commands.SpinWheel;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ThreeCellAuto;
import frc.robot.commands.ThreeCellAuto2;
import frc.robot.commands.TurnPID;
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
  public final Chassis m_chassis = new Chassis();
  public final Climber m_climber = new Climber();
  public final Intake m_intake = new Intake();
  public final Shooter m_shooter = new Shooter();
  public final Turret m_turret = new Turret();
  //public final Drive m_drive = new Drive();
  
  // Joysticks
  private  XboxController manipulatorJoystick = new XboxController(0);
  private XboxController drivingJoystick1 = new XboxController(1);
  private XboxController extraJoystick = new XboxController(3);
  
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
  private Button manipulatorStart = new JoystickButton(manipulatorJoystick, 8);
  private Button manipulatorL3 = new JoystickButton(manipulatorJoystick, 9);//Joystick press
  private Button manipulatorR3 = new JoystickButton(manipulatorJoystick, 10);//Joystick press

  // Triggers for inputs that are not buttons
  private Trigger climbTriggerL = new Trigger((BooleanSupplier)() -> drivingJoystick1.getTriggerAxis(Hand.kLeft) > .5);
  private Trigger climbTriggerR = new Trigger((BooleanSupplier)() -> drivingJoystick1.getTriggerAxis(Hand.kRight) > .5);
  private Trigger advancePosition = new Trigger((BooleanSupplier)() -> manipulatorJoystick.getPOV() == 0);
  private Trigger reteatPosition = new Trigger((BooleanSupplier)() -> manipulatorJoystick.getPOV() == 180);
  public Trigger hoodAdjust = new Trigger((BooleanSupplier)() -> advancePosition.get() || reteatPosition.get());

  public Button extraLB = new JoystickButton(extraJoystick, 5);
  public Button extraRB = new JoystickButton(extraJoystick, 6);

  // Commands
  private final MotionMagic c_MotionMagic = new MotionMagic(m_chassis, -180, m_intake);
  private final SixCellAuto m_sixCellAuto = new SixCellAuto(m_chassis, m_intake, m_shooter, m_turret);
  //private final Music m_music = new Music(m_chassis, m_shooter, "TD.chrp");
  private final EightCellAuto m_eightCellAuto = new EightCellAuto(m_chassis, m_intake, m_shooter, m_turret);
  private final ThreeCellAuto m_threeCellAuto = new ThreeCellAuto(m_chassis, m_intake, m_shooter, m_turret);
  private final SixCellAuto2 m_sixCellAuto2 = new SixCellAuto2(m_chassis, m_intake, m_shooter, m_turret);
  private final ThreeCellAuto2 m_threeCellAuto2 = new ThreeCellAuto2(m_chassis, m_intake, m_shooter, m_turret);
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
    
    m_chassis.setDefaultCommand(new TankDrive(m_chassis, drivingJoystick1, driverRB));
    
    //m_shooter.setDefaultCommand(new Music(m_shooter, ""));
    m_turret.setDefaultCommand(new Hood2(m_turret, Constants.hoodLocate[m_turret.hoodLocate]));
    
    // Button Setup
    //  Driver Buttons
   // driverA.whenPressed(new Hood(0, m_turret));
   // driverB.whenPressed(new Hood(100, m_turret));
    driverLB.whenPressed(new switchDirection(m_chassis));
    //driverBack.whileHeld(new Climb(m_climber, 0.5));
    //driverStart.whileHeld(new Climb(m_climber, 0.75));

    climbTriggerL.and(climbTriggerR).whileActiveOnce(new Climb(m_climber, 1, m_intake));

    // Manipulator Buttons
    manipulatorA.whileHeld(new HopperOuttake(m_shooter));
    manipulatorB.whileHeld(new IntakeAndHopper(m_intake, m_shooter));
    manipulatorX.whileHeld(new IntakeInward(m_intake));
    manipulatorY.whileHeld(new DeployIntake(m_intake));
    manipulatorLimeLB.whileHeld(new HoodAndLime(m_turret));//TurretLimelight(m_turret));
    manipulatorRB.whileHeld(new SpinWheel(m_shooter));//AutoShootVelocity(m_shooter, m_turret, 5000));//
    manipulatorL3.whileHeld(new TurretTurn(m_turret, .5));
    manipulatorR3.whileHeld(new TurretTurn(m_turret, -.5));
    manipulatorStart.whileHeld(new IntakeOutward(m_intake));
    advancePosition.whenActive(new ShootToggle(m_shooter, m_turret, false));
    reteatPosition.whenActive(new ShootToggle(m_shooter, m_turret, true));
    hoodAdjust.whileActiveContinuous(new Hood2(m_turret, Constants.hoodLocate[m_turret.hoodLocate]));

    if (Constants.DEBUG) SmartDashboard.putData("Hood 1", new Hood2(m_turret, Constants.hoodLocate[0]));
    if (Constants.DEBUG) SmartDashboard.putData("Hood 4",new Hood2(m_turret, Constants.hoodLocate[3]));

    // SmartDashboard Buttons
    SmartDashboard.putData(new UnClimb(m_climber));
    //SmartDashboard.putData("TDrift", new Music(m_chassis, m_shooter, "TD.chrp"));
    if (Constants.DEBUG) SmartDashboard.putData(new ShootToggle(m_shooter, m_turret, false));
    if (Constants.DEBUG) SmartDashboard.putData("shootRetreat", new ShootToggle(m_shooter, m_turret, true));

    //Triggers
    //boolean test = drivingJoystick1.getTriggerAxis(Hand.kLeft) > .5;
    //climbTrigger.;

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_sixCellAuto2;
  }*/

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(
    //         new Translation2d(1, 1),
    //         new Translation2d(2, -1)
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     // Pass config
    //     config
    // );
        String trajectoryJSON = m_chassis.camera2Path();
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_chassis::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_chassis::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_chassis::tankDriveVolts,
        m_chassis
    );

    // Reset odometry to the starting pose of the trajectory.
    m_chassis.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_chassis.tankDriveVolts(0, 0));
  }
}
