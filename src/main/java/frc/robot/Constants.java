// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Ports on the driver station where the contollers are connected
    public static final int DRIVER_CONTROLLER = 0;
    public static final int MANIPULATOR_CONTROLLER = 1;

  //CAN ID's
    public static final int RIGHT_FRONT_DRIVE_MOTOR = 1;
    public static final int RIGHT_REAR_DRIVE_MOTOR = 2;
    public static final int LEFT_FRONT_DRIVE_MOTOR = 3;
    public static final int LEFT_REAR_DRIVE_MOTOR = 4;
    public static final int VERTICALCONVEYORMOTOR = 7;
    public static final int PNEUMATIC_CONTROL_MODULE = 9;
    public static final int SHOOTER_MOTOR1 = 15;
    public static final int HORIZONTALCONVEYORMOTOR = 17;  
    public static final int INTAKE_MOTOR = 20;
    public static final int CLIMBER_MOTOR1 = 25;
    public static final int CLIMBER_MOTOR2 = 30;

  // Tables for shooting speeds vs distance
  // If balls aren't making it in, CHECK AIR PRESSURE FOR BALLS! 3.5%
    public static final double[] SHOOTER_SPEEDS_RPM_ARRAY = {
      4500, 4800, 5180, 5600, 6300
     };
          
    public static final double[] TARGET_DISTANCE_INCHES_ARRAY = {
      105, 114, 124, 134, 142
     };


  // Drivetrain slew rate limiters
    public static final double SLEW_RATE_LIMIT_ROTATE = 8;
    public static final double SLEW_RATE_LIMIT_ACCEL = 3.8;  

  //Drivetrain auton speeds
    public static final double AUTON_SPEED = 0.41;

  // Solenoid Mappings
     //phub ports
    public static final int DRVTRN_SOL_FWD_CHN = 15;
    public static final int DRVTRN_SOL_RVS_CHN = 0;  
    public static final int INTAKE_SOLENOID_FWD = 1;
    public static final int INTAKE_SOLENOID_RVS = 2; 
    public static final int CLIMBER2_SOL_FWD = 8;
    public static final int CLIMBER2_SOL_RVS = 9; 

  // Shooter motor PID constants for velocity control
    public static final double SHOOTER_Kf = 0.051;
    public static final double SHOOTER_Kp = 0.08;
    public static final double SHOOTER_Ki = 0;
    public static final double SHOOTER_Kd = 0.6;
    public static final int SHOOTER_PID_SLOT = 0;
    public static final int kTimeoutMs = 30; // Used for all PID loops

  // Climber 1 PID constants for position control
    public static final double CLIMBER1_Kf = 0.051;
    public static final double CLIMBER1_Kp = 0.05;
    public static final double CLIMBER1_Ki = 0;
    public static final double CLIMBER1_Kd = 0.;
    public static final int CLIMBER1_PID_SLOT = 0;

  // Climber 2 PID constants for position control
    public static final double CLIMBER2_Kf = 0.051;
    public static final double CLIMBER2_Kp = 0.05;
    public static final double CLIMBER2_Ki = 0;
    public static final double CLIMBER2_Kd = 0.5;
    public static final int CLIMBER2_PID_SLOT = 0;

  // Vision Values
    public static final float LIMELIGHT_VERTICAL_ANGLE = (float) 44; //change
    public static final float LIMELIGHT_HEIGHT = (float) 21; //change, inches         
    public static final float TARGET_HEIGHT = (float) 104; //change, inches            

  // Drivetrain PID parameters
    public static final int DRIVETRAIN_MOTOR_PID_LOOP = 0;
    public static final int DRIVETRAIN_MOTOR_PID_TIMEOUT = 0;

  // Default Motor Speeds
  public static final double DEFAULT_INTAKE_OUTPUT = -1;
  public static final double DEFAULT_CLIMBER_OUTPUT = 0.15; // The output should be 75% but climbers are currently in development
  public static final double DEFAULT_HORIZONTAL_CONVEYOR_OUTPUT = 1;
  public static final double DEFAULT_VERTICAL_CONVEYOR_OUTPUT = 0.8;

  public static class CandleConstants {
    public static final int CANDLE_1_ID = 50;
    public static final int CANDLE_1_LED_COUNT = 185;
  }
}

