// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_shooterMotor1 = new WPI_TalonFX(Constants.SHOOTER_MOTOR1);
  private final WPI_TalonFX m_shooterMotor2 = new WPI_TalonFX(Constants.SHOOTER_MOTOR2);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooterMotor(double inputCommand){
    m_shooterMotor1.set(inputCommand);
    m_shooterMotor2.set(inputCommand);
  }
}
