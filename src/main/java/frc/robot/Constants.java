/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
  public static final class DriveConstants {
    public static final int LEFT_DRIVE_MASTER = 0;

    public static final int LEFT_DRIVE_SLAVE = 2;

    public static final int RIGHT_DRIVE_MASTER = 1;

    public static final int RIGHT_DRIVE_SLAVE = 3;

    public static final double MOTOR_COMPENSATION_LEFT = .02;

    // on right side so we reverse it if we put the encode on correct. disble if we
    // need to once we test
    public static final boolean LEFT_ENCODER_REVERSED = false;

    public static final boolean RIGHT_ENCODER_REVERSED = true;

    //count per revolution
    public static final int ENCODER_CPR = 4096;

    // 1/4096
    public static final double POSITION_FOR_ENCODER = 0.00024414062;

    //10/4096
    public static final double ANGULAR_VELOCITY_FOR_ENCODER = 0.00244140625;

    //this changes along with wear. remeasure at comp
    public static final double WHEEL_DIAMETER = 6.25;

    public static final double WHEEL_RADIUS = 3.125;
    
    //changes with gears. pinio>big gear * little gear>big botton gear = the value below
	  public static final double GEAR_RATIO = 9.469;
  }

  public static final class HatchConstants {
    // pcm
    public static final int SOLENOID_PORT = 0;
    public static final int[] HATCH_SOLENOID_PORTS = new int[] { 0, 1 };
  }

  public static final class AutoConstants {
    public static final double kAutoDriveDistanceInches = 60;
    public static final double kAutoBackupDistanceInches = 20;
    public static final double AUTO_DRIVE_MAX_SPEED = 0.5;
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int AUXDRIVER_CONTROLLER_PORT = 1;
  }

  public static final class ShooterConstants {
    // victor
    public static final int SHOOTER_MASTER = 0;
    public static final int SHOOTER_SLAVE = 1;

  }

  public static final class IntakeConstants {
    // victor
    public static final int INTAKE_MASTER = 2;
  }

  public static final class XboxConstants {

    // Button mappings
    static public int D_PAD = 0;
    static public int A_BUTTON = 1;
    static public int B_BUTTON = 2;
    static public int X_BUTTON = 3;
    static public int Y_BUTTON = 4;
    static public int LB_BUTTON = 5;
    static public int RB_BUTTON = 6;
    static public int BACK_BUTTON = 7;
    static public int START_BUTTON = 8;
    static public int LEFT_AXIS = 9;
    static public int RIGHT_AXIS = 10;

    // Axis control mappings
    // Notes:
    // - Left and right trigger use axis 3
    // - Left trigger range: 0 to 1
    // - Right trigger range: 0 to -1).
    static public int LEFT_AXIS_X = 6;
    static public int LEFT_AXIS_Y = 1;
    static public int LEFT_TRIGGER_AXIS = 2;
    static public int RIGHT_TRIGGER_AXIS = 3;
    static public int RIGHT_AXIS_X = 4;
    static public int RIGHT_AXIS_Y = 5;

    // Direction pad lookup angles
    static public int POV_UP = 0;
    static public int POV_RIGHT = 90;
    static public int POV_DOWNN = 180;
    static public int POV_LEFT = 270;

    
  }
  public static final class PIDConstants{
      public static final double PROPORTION = 9.31;
      public static final double INTEGRAL = 0;
      public static final double DERIVATIVE = 0;

      public static final double KS_FEEDFOWARD = 1.1;
      public static final double KV_FEEDFOWARD = 0.329;
      public static final double KA_FEEDFOWARD = 0.0933; 
      public static final double OPTIMAL_KP = 9.31;
      public static final double OPTIMAL_KD = 4.51;
      //PID STUFF
    }
}
