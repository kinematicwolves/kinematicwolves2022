// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
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
  private final int encoderCountsPerRev = 2048;

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftFront, m_leftRear);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightFront, m_rightRear);

  private final ADIS16448_IMU imu = new ADIS16448_IMU();
  
  private final DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
    Units.inchesToMeters(Constants.TRACK_WIDTH_INCHES)
    );
  
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
  getGyroHeading(), new Pose2d(0, 0, new Rotation2d()));
    
  /** Creates a new DifferentialDrivetrain. */
  public DifferentialDrivetrain() {
  }

  public Rotation2d getGyroHeading(){
    return Rotation2d.fromDegrees(-1 * imu.getAngle());
  }

  public double getWheelSpeedMetersPerSecond(WPI_TalonFX motorController){
    double rawSpeed = motorController.getSelectedSensorVelocity(); // raw sensor units per 100ms
    double wheelSpeedRPM = rawSpeed / encoderCountsPerRev * 1000 * 60; // RPM, (sensor units / 100ms)(rev / sensor units)(ms / s) (s / min)
    // NOTE: NEED TO USE THE ACTIVE GEAR RATIO HERE or the speeds won't be right
    // (Rev/min)(2pi rad/rev) (radius)
    return wheelSpeedRPM * (2 * Math.PI)  * Units.inchesToMeters(Constants.WHEEL_RADIUS_INCHES) / 60;

  }

  public DifferentialDriveWheelSpeeds getDifferentialDriveWheelSpeeds(){
    double leftWheelSpeed = getWheelSpeedMetersPerSecond(m_leftFront);
    double rightWheelSpeed = getWheelSpeedMetersPerSecond(m_rightFront);
    return new DifferentialDriveWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
  }

  private void configureDriveMotorFeedback() {
    m_leftFront.configFactoryDefault();

    m_leftFront.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.DRIVETRAIN_MOTOR_PID_LOOP,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);
    
    m_rightFront.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);

    m_rightFront.configFactoryDefault();
    m_rightFront.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor, Constants.DRIVETRAIN_MOTOR_PID_LOOP,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);
    
    m_rightFront.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero,
        Constants.DRIVETRAIN_MOTOR_PID_TIMEOUT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var wheelSpeeds = getDifferentialDriveWheelSpeeds();
    SmartDashboard.putNumber("IMU Angle reading (deg)", -1 * imu.getAngle());
    SmartDashboard.putNumber("Left wheel speed (m/s)", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right wheel speed (m/s)", wheelSpeeds.rightMetersPerSecond);

  }


  private double logAdjustment (double x) {
    if(Math.abs(x)>0.7)
    return x;
    double x1=x*100/127;
    return x1;
  }
  public void moveWithJoysticks(XboxController driverController) {

    // Get axis values for speed and rotational speed
    /*double xSpeed = driverController.getLeftY();
    //double zRotation_rate = -1 * driverController.getLeftX();
    double zRotation_rate = -1 * driverController.getRightX(); //for POV Drive
*/
    //hack hack hack fix this
    double xSpeed = logAdjustment (driverController.getRightX());
    //double zRotation_rate = -1 * driverController.getLeftX();
    double zRotation_rate = logAdjustment(1 * driverController.getLeftY()); //for POV Drive

    accelerationFilter.calculate(xSpeed);
    rotationFilter.calculate(zRotation_rate);

    // Drive Robot with commanded linear velocity and yaw rate commands
    drive.arcadeDrive(xSpeed, zRotation_rate);

    SmartDashboard.putNumber("X speed commanded by driver", driverController.getLeftY());
    SmartDashboard.putNumber("zRotation Rate Commanded by driver", driverController.getLeftX());
  }
  public void move_backward(double speed){

    // Simple call to arcade drive to move along a straight line at a constant speed
    drive.arcadeDrive(speed, -0);

  }
}
