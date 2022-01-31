// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_shooterMotor1 = new WPI_TalonFX(Constants.SHOOTER_MOTOR1);
  private final int encoderCountsPerRev = 2048;
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    configureMotorFeedback(m_shooterMotor1);
  }

  public void configureMotorFeedback(WPI_TalonFX motor){
    m_shooterMotor1.configFactoryDefault();
		m_shooterMotor1.setNeutralMode(NeutralMode.Brake);
    m_shooterMotor1.setInverted(TalonFXInvertType.Clockwise);
		/* Config neutral deadband to be the smallest possible */
		m_shooterMotor1.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
    m_shooterMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.SHOOTER_PID_SLOT, Constants.kTimeoutMs);
											

		/* Config the peak and nominal outputs */
		m_shooterMotor1.configNominalOutputForward(0, Constants.kTimeoutMs);
		m_shooterMotor1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		m_shooterMotor1.configPeakOutputForward(1, Constants.kTimeoutMs);
		m_shooterMotor1.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		m_shooterMotor1.config_kF(Constants.SHOOTER_PID_SLOT, Constants.SHOOTER_Kf, Constants.kTimeoutMs);
		m_shooterMotor1.config_kP(Constants.SHOOTER_PID_SLOT, Constants.SHOOTER_Kp, Constants.kTimeoutMs);
		m_shooterMotor1.config_kI(Constants.SHOOTER_PID_SLOT, Constants.SHOOTER_Ki, Constants.kTimeoutMs);
		m_shooterMotor1.config_kD(Constants.SHOOTER_PID_SLOT, Constants.SHOOTER_Kd, Constants.kTimeoutMs);
		
  }

  public double getMotorSpeedRPM(WPI_TalonFX motorController){
    double rawSpeed = motorController.getSelectedSensorVelocity(); // raw sensor units per 100ms
    double motorSpeedRPM = rawSpeed / encoderCountsPerRev * 1000 * 60; // RPM, (sensor units / 100ms)(rev / sensor units)(ms / s) (s / min)
    return motorSpeedRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter motor output: ", m_shooterMotor1.getMotorOutputPercent());
    SmartDashboard.putNumber("Shooter motor speed (units per 100ms)", m_shooterMotor1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter speed (RPM)", getMotorSpeedRPM(m_shooterMotor1));
  }

  public void runShooterMotor(double inputCommand){
    m_shooterMotor1.set(inputCommand);
  }

  public double convertToNativeSpeedCountsPer100ms(double speedRPM){
    return speedRPM * encoderCountsPerRev /  (1000 * 60) * 100;
  }

  public void setShooterMotorSpeed(double speedRPM){
    // Set in units per 100 ms
    m_shooterMotor1.set(TalonFXControlMode.Velocity, convertToNativeSpeedCountsPer100ms(speedRPM));
  }
}
