/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Driving Motors 
    public static final int LeftLeader = 14; 
    public static final int LeftFollower = 15; 
    public static final int RightLeader = 1; 
    public static final int RightFollower = 16; // PDP port 0

    //Shooter Motors 
    public static final int RightShooter = 2; 
    public static final int LeftShooter = 3; 
    public static final int HoodAngle = 4; 
    public static final int TurretRotate = 5; 
    //Limelight does not get its own motor port, but it is plugged into PDP

    //Hopper Motors 
    public static final int HopperRight = 9; 
    public static final int HopperLeft = 6;   
    public static final int Feed = 10; 

    //Intake Motors
    public static final int RightIntake = 7; 
    public static final int LeftIntake = 8; 

    //Control Panel 
    public static final int ControlPanel = 11; 
  
    //Climbing 
    public static final int Climber = 13; 
//Gains stuff
	public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
	public final static Gains kGains_Turning = new Gains( 2.0, 0.0,  4.0, 0.0,            200,  1.00 );
	public final static Gains kGains_Velocit = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
	public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 );
  
    public final static double kNeutralDeadband = 0.001;

    public final static int pidgey = 22; 
    public final static int kPigeonUnitsPerRotation = 8192; 
    public final static int kTimeoutMs = 30; 
// these numbers might not be correct. I just copied from online so they might need to be changed
}
