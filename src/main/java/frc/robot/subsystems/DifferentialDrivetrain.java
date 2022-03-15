// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class DifferentialDrivetrain extends SubsystemBase {
  // Initial 13:42
  // Low Gear 14:60
  // High Gear 24:50
  public SlewRateLimiter rotationFilter = new SlewRateLimiter(Constants.SLEW_RATE_LIMIT_ROTATE);
  public SlewRateLimiter accelerationFilter = new SlewRateLimiter(Constants.SLEW_RATE_LIMIT_ACCEL);

  private final WPI_TalonFX m_leftFront = new WPI_TalonFX(Constants.LEFT_FRONT_DRIVE_MOTOR);
  private final WPI_TalonFX m_leftRear = new WPI_TalonFX(Constants.LEFT_REAR_DRIVE_MOTOR);
  private final WPI_TalonFX m_rightFront = new WPI_TalonFX(Constants.RIGHT_FRONT_DRIVE_MOTOR);
  private final WPI_TalonFX m_rightRear = new WPI_TalonFX(Constants.RIGHT_REAR_DRIVE_MOTOR);


  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftFront, m_leftRear);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightFront, m_rightRear);

  private final DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  public static final double kMaxSpeed = 2.7; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;

  private boolean speedLimited = false;

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.0)); // Distance
                                                                                                                // between
                                                                                                                // two
                                                                                                                // sets
                                                                                                                // of
                                                                                                                // wheels
                                                                                                                // (inches)

  /** Creates a new DifferentialDrivetrain. */
  public DifferentialDrivetrain() {
  }

  private void configureDriveMotorFeedback() {
    m_leftFront.configFactoryDefault();

    m_leftFront.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.DRIVETRAIN_MOTOR_PID_LOOP,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);
    m_leftFront.config_kF(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_leftFront.config_kP(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_leftFront.config_kI(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_leftFront.config_kD(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);

    m_rightFront.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);

    m_rightFront.configFactoryDefault();
    m_rightFront.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.DRIVETRAIN_MOTOR_PID_LOOP,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);
    m_rightFront.config_kF(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_rightFront.config_kP(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_rightFront.config_kI(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);
    m_rightFront.config_kD(Constants.DRIVETRAIN_MOTOR_PID_LOOP, 0);

    m_rightFront.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  private double logAdjustment (double x) {
    return (Math.abs(x) > 0.7) ? x : (x * 100 /127);
  }

  public void enableSpeedLimit(){
    speedLimited = true;
  }

  public void disableSpeedLimit(){
    speedLimited = false;
  }

  public boolean isSpeedLimited(){
    return speedLimited;
  }

  public void moveWithJoysticks(XboxController driverController) {
    double xSpeed = logAdjustment (driverController.getRightX());
    
    double zRotationRate = logAdjustment(1 * driverController.getLeftY()); //for POV Drive
    if (speedLimited){
      xSpeed *= 0.4;
      zRotationRate *= 0.4;

    }
    // Drive Robot with commanded linear velocity and yaw rate commands
    drive.arcadeDrive(accelerationFilter.calculate(xSpeed), rotationFilter.calculate(zRotationRate));

    SmartDashboard.putNumber("X speed commanded by driver", accelerationFilter.calculate(xSpeed));
    SmartDashboard.putNumber("Rotation command", rotationFilter.calculate(zRotationRate));
  }
  public void moveBackward(double speed){

    // Simple call to arcade drive to move along a straight line at a constant speed
    drive.arcadeDrive(0, -1 * speed);

  }

  // Drivetrain Variables
  public boolean isHighGear = false; // Initialize to low gear


  public void shiftToHighGear(PneumaticSubsystem pneumaticSubsystem) {
    pneumaticSubsystem.setDrivetrainSolenoidFoward();
    // Set solenoid switch to forward
    isHighGear = true;
    
	}

  public void shiftToLowGear(PneumaticSubsystem pneumaticSubsystem) {
    pneumaticSubsystem.setDrivetrainSolenoidReverse();
    // Set solenoid switch to reverse
    isHighGear = false;
    
	}

  public boolean isInHighGear(){
    return isHighGear; 
    
  }
}
