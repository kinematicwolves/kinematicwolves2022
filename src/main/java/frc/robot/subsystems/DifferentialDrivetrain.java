// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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

  private String currentGear = "high"; // Default starting gear
  private double INITIAL_GEAR_RATIO = 13/42;
  private double LOW_GEAR_RATIO = 14/60;
  private double HIGH_GEAR_RATIO = 24/50;

  private static final double wheelRadiusInches = 6;
  private static final double alignWindow = 2; 
  private static final double trackWidthInches = 27;

  private boolean speedLimited = false;

  // private final ADIS16448_IMU imu = new ADIS16448_IMU();
  
  private final DifferentialDrive drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
    Units.inchesToMeters(trackWidthInches)
    );
  
  // DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
  // getGyroHeading(), new Pose2d(0, 0, new Rotation2d()));
    
  /** Creates a new DifferentialDrivetrain. */
  public DifferentialDrivetrain() {
    m_rightFront.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    m_leftFront.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
  }

  public double getCurrentGearRatio(){
    return currentGear == "high" ? HIGH_GEAR_RATIO : LOW_GEAR_RATIO;
  }

  public void setLowGear(){
    currentGear = "low";
  }

  public void setHighGear(){
    currentGear = "high";
  }

  public boolean isHighGear(){
    return currentGear == "high";
  }

  // public Rotation2d getGyroHeading(){
  //   return Rotation2d.fromDegrees(-1 * imu.getAngle());
  // }

  public double getWheelSpeedMetersPerSecond(WPI_TalonFX motorController){
    double rawSpeed = motorController.getSelectedSensorVelocity(); // raw sensor units per 100ms
    double wheelSpeedRPM = rawSpeed / encoderCountsPerRev * 1000 * 60 * getCurrentGearRatio() * INITIAL_GEAR_RATIO; // RPM, (sensor units / 100ms)(rev / sensor units)(ms / s) (s / min)
    // (Rev/min)(2pi rad/rev) (radius)
    return wheelSpeedRPM * (2 * Math.PI) * Units.inchesToMeters(wheelRadiusInches) / 60;

  }

  public DifferentialDriveWheelSpeeds getDifferentialDriveWheelSpeeds(){
    double leftWheelSpeed = getWheelSpeedMetersPerSecond(m_leftFront);
    double rightWheelSpeed = getWheelSpeedMetersPerSecond(m_rightFront);
    return new DifferentialDriveWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var wheelSpeeds = getDifferentialDriveWheelSpeeds();
    var chassisSpeed = kinematics.toChassisSpeeds(wheelSpeeds);
    // SmartDashboard.putNumber("IMU Angle reading (deg)", -1 * imu.getAngle());
    SmartDashboard.putNumber("Linear velocity (vx) (m/s)", chassisSpeed.vxMetersPerSecond);
    SmartDashboard.putNumber("Rotational speed (RPM)", chassisSpeed.omegaRadiansPerSecond / (2 * Math.PI) * 60);
    SmartDashboard.putNumber("Distance driven - inches (auton fwd)", countsToDistanceDrivenInches(m_rightFront.getSelectedSensorPosition()));
  }

  public double countsToDistanceDrivenInches(double counts){
    return counts / encoderCountsPerRev * getCurrentGearRatio() * 2 * Math.PI * wheelRadiusInches;
  }

  public double getXDistanceDrivenInches(){
    return countsToDistanceDrivenInches(m_leftFront.getSelectedSensorPosition());
  }

  public void driveForward( double speed){
    // This assumes motion ONLY in the x direction!
    drive.arcadeDrive(0, speed);
  
  }

  private double logAdjustment (double x) {
    return (Math.abs(x) > 0.7) ? x : (x * 100 /127);
  }

  public void setMotorsBrake(){
    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_leftRear.setNeutralMode(NeutralMode.Brake);
    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_rightRear.setNeutralMode(NeutralMode.Brake);
  }

  public void setMotorsCoast(){
    m_leftFront.setNeutralMode(NeutralMode.Coast);
    m_rightFront.setNeutralMode(NeutralMode.Coast);
    m_leftRear.setNeutralMode(NeutralMode.Coast);
    m_rightRear.setNeutralMode(NeutralMode.Coast);
  }

  public void driveForwardDistance(double distance){

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

  public boolean isLinedUp(VisionSubsystem visionSubsystem){
    var horizalAngle = visionSubsystem.getFilteredHorizontalAngle();
    // degrees
    return (horizalAngle < alignWindow) & (horizalAngle > (-1 * alignWindow));
  }

  public void rotateClockwise(double rotateSpeed){
    drive.arcadeDrive(rotateSpeed, 0);
  }

  public void rotateDrivetrainToTarget(double rotateSpeed, VisionSubsystem visionSubsystem){
    // This assumes the limelight has a target
    var horizalAngle = visionSubsystem.getFilteredHorizontalAngle();
    if (isLinedUp(visionSubsystem)){
      // Lined up!
      drive.arcadeDrive(0, 0);
    }
    else if (horizalAngle < (-1 * alignWindow)){
      // rotate clockwise
      drive.arcadeDrive(-1 * rotateSpeed, 0);
    }
    else if (horizalAngle > alignWindow){
      // rotate counter clockwise
      drive.arcadeDrive(rotateSpeed, 0);
    }
  }
}
