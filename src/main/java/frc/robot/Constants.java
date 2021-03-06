/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean DEBUG = true;

    //Driving Motors 
    public static final int LeftLeader = 14; 
    public static final int LeftFollower = 15; 
    public static final int RightLeader = 1; 
    public static final int RightFollower = 16; // PDP port 0

    //Shooter Motors 
    public static final int RightShooter = 2; 
    public static final int LeftShooter = 3; 
    public static final int HoodAngle = 13; 
    public static final int TurretRotate = 5; 
    //Limelight does not get its own motor port, but it is plugged into PDP

    //Hopper Motors 
    public static final int HopperRight = 9; 
    public static final int HopperLeft = 6;   
    public static final int Feed = 10; 
    public static final int FeedTwo = 12;

    //Intake Motors
    public static final int RightIntake = 7; 
    public static final int LeftIntake = 8; 

    //Control Panel 
    public static final int ControlPanel = 11; 
  
    //Climbing 
    public static final int Climber = 78;//changed because of conflict in PDP slots (would have been 12)

    //Gains stuff
    public final static Gains kGains_Distanc = new Gains( 0.2, 0.0,  0.0, 0.2,            100,  1.00 );
    public final static Gains kGains_Turning = new Gains( 2.0, 0.0,  4.0, 0.0,            200,  1.00 );
    public final static Gains kGains_Velocit = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
    public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 );
  
    public final static double kNeutralDeadband = 0.001;

    public final static int pidgey = 22; 
    public final static int kPigeonUnitsPerRotation = 8192; 
    public final static int kTimeoutMs = 30; 
	
    // Shooter and hood constants
    public final static int [] shooterSpeed = {3800, 4500, 5000, 5700, 6000};
    public final static int [] hoodLocate = {-28000, -31000, -37000, -50000, -58000};
    public final static double [] feedGain = {1, 1, 0.85, 0.4, 0.4};

    // Deadbands
    public final static double kVelocityDeadband = ((100 * 2048) / 600); // 100 RPM to Click Per 100ms
    // these numbers might not be correct. I just copied from online so they might need to be changed


    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
