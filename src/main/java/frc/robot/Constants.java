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

    // Drivetrain subsystem controllers
    public static final int RIGHT_FRONT_DRIVE_MOTOR = 1;
    public static final int RIGHT_REAR_DRIVE_MOTOR = 2;
    public static final int LEFT_FRONT_DRIVE_MOTOR = 3;
    public static final int LEFT_REAR_DRIVE_MOTOR = 4;

    // Drivetrain PID parameters
    public static final int DRIVETRAIN_MOTOR_PID_LOOP = 0;
    public static final int DRIVETRAIN_MOTOR_PID_TIMEOUT = 0;

    // Drivetrain rate limiters
    // Slew rate limiters
    public static final double SLEW_RATE_LIMIT_ROTATE = 0.5;
    public static final double SLEW_RATE_LIMIT_ACCEL = 0.5;

    //motor speeds
    public static final double AUTON_SPEED = 0.3;

    // Intake Motor
    public static final int INTAKE_MOTOR = 20;
    public static final double DEFAULT_INTAKE_OUTPUT = 0.9;

    // Pnuematic 
    public static final int INTAKE_SOLENOID_FWD = 0;
    public static final int INTAKE_SOLENOID_RVS = 1; 

    public static final int PNEUMATIC_CONTROL_MODULE = 9;

   // Solenoid Mappings
    public static final int DRVTRN_SOL_FWD_CHN = 2;
    public static final int DRVTRN_SOL_RVS_CHN = 3;

    //Shooter Motor can id's
    public static final int SHOOTER_MOTOR1 = 15;

    // Shooter motor PID constants for velocity control
    public static final double SHOOTER_Kf = 0.051;
    public static final double SHOOTER_Kp = 0.05;
    public static final double SHOOTER_Ki = 0;
    public static final double SHOOTER_Kd = 0.5;
    public static final int SHOOTER_PID_SLOT = 0;
    public static final int kTimeoutMs = 30;
    

    //Conveyor Motor can id's
    public static final int HORIZONTALCONVEYORMOTOR = 16;
    public static final int VERTICALCONVEYORMOTOR = 17;
    
}
