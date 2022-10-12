// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_climberMotor1 = new WPI_TalonFX(Constants.CLIMBER_MOTOR1);
  private final WPI_TalonFX m_climberMotor2 = new WPI_TalonFX(Constants.CLIMBER_MOTOR2);
  private final int encoderCountsPerRev = 2048;
  private String climber1State = "Initial Position";
  private String climber2State = "Initial Position";
  private final int GEAR_REDUCTION = 25 / 1;
  private final double SPOOL_CIRCUMFERENCE_INCHES = 3.1; // Inches
  private boolean climber2IsDeployed = false; 

  /* 
    These are a constant offset to gravity. Set such that it retains a position of zero. This
    is an arbitrary output that is always added to the motor output.
  /* 
    Constants for extending climber
    25:1 gear ratio
    3.1" spool circumference
  */

  // These need to be adujsted 
  private final double MINIMUM_DISTANCE_C1 = 0;
  private final double MAXIMUM_DISTANCE_C1 = 83.8; // UNITS INCHES
  private final double WINDOW_THRESHOLD_C1 = 0.2; // UNITS INCHES + I dont know what this means
  private final double CLIMBER1_HEIGHT_C1 = 41.9; // UNITS INCHES

  private final double MINIMUM_DISTANCE_C2 = 0;
  private final double MAXIMUM_DISTANCE_C2 = 47.2; // UNITS INCHES
  private final double WINDOW_THRESHOLD_C2 = 0.2; // UNITS INCHES + I dont know what this means
  private final double CLIMBER2_HEIGHT = 23.6; // UNITS INCHES


  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climberMotor1.setInverted(TalonFXInvertType.CounterClockwise); //Change to clockwise if motor is spinning in the wrong direction
    m_climberMotor2.setInverted(TalonFXInvertType.Clockwise); //Change to counterclockwise if motor is spinning in the wrong direction 

    // No idea what the soft limit threshold does
    // bruh why are there 2 things making the motor start at 0 (espesially the one with the timeoutMs parameter)??
    m_climberMotor1.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, 10);
    m_climberMotor1.setSelectedSensorPosition(0);
    m_climberMotor1.configForwardSoftLimitThreshold(convertPositionInchesToCounts(MAXIMUM_DISTANCE_C1)); 
    m_climberMotor1.configForwardSoftLimitEnable(true);

    m_climberMotor1.configReverseSoftLimitThreshold(0);
    m_climberMotor1.configReverseSoftLimitEnable(true);

    m_climberMotor2.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, 10);
    m_climberMotor2.setSelectedSensorPosition(0);
    m_climberMotor2.configForwardSoftLimitThreshold(convertPositionInchesToCounts(MINIMUM_DISTANCE_C2)); 
    m_climberMotor2.configForwardSoftLimitEnable(true);

    m_climberMotor2.configReverseSoftLimitThreshold(0);
    m_climberMotor2.configReverseSoftLimitEnable(true);
  }

  private void monitorClimber1State(){
    // Using a string to represent the state of climber 1 is a hack implementation, but oh well.
    if (getPositionInches() < MINIMUM_DISTANCE_C1 + WINDOW_THRESHOLD_C1){
      climber1State = "Initial Position";
    }
    else if ((getPositionInches() > MINIMUM_DISTANCE_C1 + WINDOW_THRESHOLD_C1) & (getPositionInches() < WINDOW_THRESHOLD_C1 + CLIMBER1_HEIGHT_C1)){
      climber1State = "Raising To Climb";
    }
    else if ((getPositionInches() > CLIMBER1_HEIGHT_C1 - WINDOW_THRESHOLD_C1) & (getPositionInches() < CLIMBER1_HEIGHT_C1 + WINDOW_THRESHOLD_C1)){
      climber1State = "Ready to climb";
    }
    else if ((getPositionInches() > CLIMBER1_HEIGHT_C1 + WINDOW_THRESHOLD_C1) & (getPositionInches() < MAXIMUM_DISTANCE_C1 - WINDOW_THRESHOLD_C1)){
      climber1State = "Climbing";
    }
    else if ((getPositionInches() > MAXIMUM_DISTANCE_C1 - WINDOW_THRESHOLD_C1) & (getPositionInches() < MAXIMUM_DISTANCE_C1)){
      climber1State = "At Max Position";
    }
    else {
      climber1State = "Out of bounds";
    }

  }

  private void monitorClimber2State(){
    // Using a string to represent the state of climber 2 is a hack implementation, but oh well.
    if (getPositionInches() < MINIMUM_DISTANCE_C2 + WINDOW_THRESHOLD_C2){
      climber2State = "Initial Position";
    }
    else if ((getPositionInches() > MINIMUM_DISTANCE_C2 + WINDOW_THRESHOLD_C2) & (getPositionInches() < WINDOW_THRESHOLD_C2 + CLIMBER2_HEIGHT)){
      climber2State = "Raising To Climb";
    }
    else if ((getPositionInches() > CLIMBER2_HEIGHT - WINDOW_THRESHOLD_C2) & (getPositionInches() < CLIMBER2_HEIGHT + WINDOW_THRESHOLD_C2)){
      climber2State = "Ready to climb";
    }
    else if ((getPositionInches() > CLIMBER2_HEIGHT + WINDOW_THRESHOLD_C2) & (getPositionInches() < MAXIMUM_DISTANCE_C2 - WINDOW_THRESHOLD_C2)){
      climber2State = "Climbing";
    }
    else if ((getPositionInches() > MAXIMUM_DISTANCE_C2 - WINDOW_THRESHOLD_C2) & (getPositionInches() < MAXIMUM_DISTANCE_C2)){
      climber2State = "At Max Position";
    }
    else {
      climber2State = "Out of bounds";
    }

  }

  public String getClimber1State(){
    return climber1State;
  }

  public String getClimber2State(){
    return climber2State;
  }

  public boolean atClimb1Position(){
    return getClimber1State() == "Ready to climb";
  }

  public boolean isSafeForClimb(){
    if ((getClimber1State() == "Initial Position" ) | (getClimber1State() == "Raising To Climb")){
      return true;
    }
    else {
      return false;
    }
  }

  public boolean isClimber1Finished(){
    return getClimber1State() == "At Max Position";
  }
  
  public boolean isSafeForSecondClimb(){
    return (getClimber1State() == "At Max Position") & ((getClimber2State() == "Initial Position" ) | (getClimber2State() == "Raising To Climb"));
  }

  public boolean isClimber2Finished(){
    return getClimber2State() == "At Max Position";
  }
  
  @Override
  public void periodic() {
    monitorClimber1State();
    monitorClimber2State();
    // This method will be called once per scheduler run
    double currentPositionClimber1 = m_climberMotor1.getSelectedSensorPosition();
    SmartDashboard.putNumber("Climber 1 counts", currentPositionClimber1);
    SmartDashboard.putNumber("Climber 1 position (inches)", convertCountsToPositionInches(currentPositionClimber1));
    SmartDashboard.putString("Climber 1 state", getClimber1State());

    double currentPositionClimber2 = m_climberMotor2.getSelectedSensorPosition();
    SmartDashboard.putNumber("Climber 2 counts", currentPositionClimber2);
    SmartDashboard.putNumber("Climber 2 position (inches)", convertCountsToPositionInches(currentPositionClimber2));
    SmartDashboard.putString("Climber 2 state", getClimber2State());

  }

  public double getPositionInches(){
    return convertCountsToPositionInches(m_climberMotor1.getSelectedSensorPosition());
  }
 
  public double convertCountsToPositionInches(double positionCounts){
    return Math.abs(positionCounts / encoderCountsPerRev * GEAR_REDUCTION  * SPOOL_CIRCUMFERENCE_INCHES);
  }

  public double convertPositionInchesToCounts(double positionInches){
    return positionInches * encoderCountsPerRev / (GEAR_REDUCTION * SPOOL_CIRCUMFERENCE_INCHES);
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