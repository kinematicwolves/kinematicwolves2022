// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
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

  /*
   * The second climber needs to be calibrated. 
   * - Fisrt make sure the 2nd climber is FULLY reset. If you dont know what this means then ask mehcanical. The hook should be fully touching the bar. And remind them that the rope has to be on top. 
   * - Then re-deploy the code. Go to driverstation and open up smart dashboard. Make sure all data is on there. Your going to be looking for "climber to position (Ash)". 
   * - Once you find it, it should say 0. If it doesn't then re-deploy the code.
   * - After you've confirmed it's at 0, tell mehcanical to set the 2nd climber to is max height. 
   * - You'll see that the numbers have changed for the 2nd climber in smart Dashboard. The number that's in there is the number your going to put in the  "CLIMBER2_HEIGHT = 0" constant below
   * - After you've copied and pasted the number tell mehcanical to fully extend the climber back down BEFORE THE RED!!!!! If it's not before the red then the climber will break:) 
   * - Once it's down, check smart dashboard to copy and paste the number you get into "MAXIMUM_DISTANCE_C2 = 0" constant below. 
   * - Now your done and can test the climbers. Just make sure the speeds are safe on them. You can change the speed by going to the Constants.java file and look for "DEFAULT_CLIMBER_OUTPUT". (Control + F if you can't find it.)
   */
  private final double MINIMUM_DISTANCE_C1 = 0;
  private final double MAXIMUM_DISTANCE_C1 = 40000; // Robot is all the way up on mid bar
  private final double WINDOW_THRESHOLD_C1 = 0.2; // Dont worry about this
  private final double CLIMBER1_HEIGHT_C1 = 21000; // Climb postion

  //Needs to be callibrated
  private final double MINIMUM_DISTANCE_C2 = 0;
  private final double MAXIMUM_DISTANCE_C2 = 10722.019389; // Robot is up on high bar
  private final double WINDOW_THRESHOLD_C2 = 0.2; // Dont worry about this
  private final double CLIMBER2_HEIGHT = 0; // Ready to be postioned and climb high bar 


  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // direction of rotation
    m_climberMotor1.setInverted(TalonFXInvertType.CounterClockwise); //Change to clockwise if motor is spinning in the wrong direction
    m_climberMotor2.setInverted(TalonFXInvertType.CounterClockwise); //Change to counterclockwise if motor is spinning in the wrong direction 

    // current limits
    // m_climberMotor1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,
    // 50, 0.5));
    // m_climberMotor2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,
    // 50, 0.5));

    m_climberMotor1.configFactoryDefault();
    m_climberMotor2.configFactoryDefault();
    
    m_climberMotor1.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, 10);
    m_climberMotor1.setSelectedSensorPosition(0);
    m_climberMotor1.configForwardSoftLimitThreshold(convertPositionInchesToCounts(MAXIMUM_DISTANCE_C1)); 
    m_climberMotor1.configForwardSoftLimitEnable(true);

    m_climberMotor1.configReverseSoftLimitThreshold(0);
    m_climberMotor1.configReverseSoftLimitEnable(true);

    m_climberMotor2.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, 10);
    m_climberMotor2.setSelectedSensorPosition(0);
    m_climberMotor2.configForwardSoftLimitThreshold(convertPositionInchesToCounts(MAXIMUM_DISTANCE_C2)); 
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
    if (getPositionInches_C2() < MINIMUM_DISTANCE_C2 + WINDOW_THRESHOLD_C2){
      climber2State = "C2_Initial Position";
    }
    else if ((getPositionInches_C2() > MINIMUM_DISTANCE_C2 + WINDOW_THRESHOLD_C2) & (getPositionInches_C2() < WINDOW_THRESHOLD_C2 + CLIMBER2_HEIGHT)){
      climber2State = "C2_Raising To Climb";
    }
    else if ((getPositionInches_C2() > CLIMBER2_HEIGHT - WINDOW_THRESHOLD_C2) & (getPositionInches_C2() < CLIMBER2_HEIGHT + WINDOW_THRESHOLD_C2)){
      climber2State = "C2_Ready to climb";
    }
    else if ((getPositionInches_C2() > CLIMBER2_HEIGHT + WINDOW_THRESHOLD_C2) & (getPositionInches_C2() < MAXIMUM_DISTANCE_C2 - WINDOW_THRESHOLD_C2)){
      climber2State = "C2_Climbing";
    }
    else if ((getPositionInches_C2() > MAXIMUM_DISTANCE_C2 - WINDOW_THRESHOLD_C2) & (getPositionInches_C2() < MAXIMUM_DISTANCE_C2)){
      climber2State = "C2_At Max Position";
    }
    else {
      climber2State = "C2_Out of bounds";
    }
  }

  public boolean isClimber1Finished(){
    return getClimber1State() == "At Max Position";
  }

  public String getClimber1State(){
    return climber1State;
  }

  public String getClimber2State(){
    return climber2State;
  }

  
  @Override
  public void periodic() {
    monitorClimber1State();
    monitorClimber2State();
    // This method will be called once per scheduler run
    double currentPositionClimber1 = m_climberMotor1.getSelectedSensorPosition();
    SmartDashboard.putNumber("Climber 1 position (Ash)", convertCountsToPositionInches(currentPositionClimber1));
    SmartDashboard.putString("Climber 1 state", getClimber1State());

    double currentPositionClimber2 = m_climberMotor2.getSelectedSensorPosition();
    SmartDashboard.putNumber("Climber 2 position (Ash)", convertCountsToPositionInches(currentPositionClimber2));
    SmartDashboard.putString("Climber 2 state", getClimber2State());
    SmartDashboard.putNumber("Supply current", m_climberMotor1.getStatorCurrent());
    SmartDashboard.putNumber("Supply current", m_climberMotor2.getStatorCurrent());
  }

  public double getPositionInches(){
    return convertCountsToPositionInches(m_climberMotor1.getSelectedSensorPosition());
  }

  public double getPositionInches_C2(){
    return convertCountsToPositionInches(m_climberMotor2.getSelectedSensorPosition());
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