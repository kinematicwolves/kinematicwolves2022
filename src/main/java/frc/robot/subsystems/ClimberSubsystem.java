// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_climberMotor1 = new WPI_TalonFX(Constants.CLIMBER_MOTOR1);
  private final WPI_TalonFX m_climberMotor2 = new WPI_TalonFX(Constants.CLIMBER_MOTOR2);
  private boolean climber2IsDeployed = false; 

  private double CLIMBER_GEAR_RATIO = 25.0/1.0;
  private final double encoderCountsPerRev = 2048;
  private final double spoolCurcumfence = 1.5;

  /* 
    These are a constant offset to gravity. Set such that it retains a position of zero. This
    is an arbitrary output that is always added to the motor output.
  */ 

public double getClimberDistanceTravledPerRotation(){
  double climberSpoolRotation = encoderCountsPerRev * CLIMBER_GEAR_RATIO;
  return climberSpoolRotation * spoolCurcumfence;
}

public ClimberDistanceTravled getClimberDistanceTravled(){
  double climber1DistanceTravled = getClimberDistanceTravledPerRotation(m_climberMotor1);

}
  
  @Override
  public void periodic() {
    m_climberMotor1.setInverted(TalonFXInvertType.CounterClockwise);

    m_climberMotor2.setInverted(TalonFXInvertType.Clockwise);

    m_climberMotor1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,
     50, 0.5));
     m_climberMotor2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,
     50, 0.5));

    //smart dashboard info
    SmartDashboard.putNumber("Sensor Position", m_climberMotor1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Supply current", m_climberMotor1.getStatorCurrent());   
    SmartDashboard.putNumber("Sensor Position", m_climberMotor2.getSelectedSensorPosition());
    SmartDashboard.putNumber("Supply current", m_climberMotor2.getStatorCurrent());

    SmartDashboard.putNumber("Climber 1 distance traveled (in inches) ", CLIMBER_GEAR_RATIO)
  }

  public void setClimberMotor1Output(double commandedOutput){
    m_climberMotor1.set(commandedOutput);
  }

  public void setClimberMotor2Output(double commandedOutput){
    m_climberMotor2.set(commandedOutput);
  }

  public boolean isClimber2Deployed(){
    return climber2IsDeployed; 
  }

  public void setClimber2Deployed(PneumaticSubsystem pneumaticSystem){
    // add solenoid as input and flip here
    climber2IsDeployed = true; 
    pneumaticSystem.setClimber2Deployed();
  }

  public void setClimber2Undeployed(PneumaticSubsystem pneumaticSubsystem){
    // Add solenoid as input and flip here
    climber2IsDeployed = false; 
    pneumaticSubsystem.setClimber2Undeployed();
  }
}
