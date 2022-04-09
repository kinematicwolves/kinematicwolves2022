// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_climberMotor1 = new WPI_TalonFX(Constants.CLIMBER_MOTOR1);
  private final WPI_TalonFX m_climberMotor2 = new WPI_TalonFX(Constants.CLIMBER_MOTOR2);
  // public static Servo angleActuator_1 = new Servo(Constants.LINEAR_ACTUATOR_1); // PWM controlled
  private final int encoderCountsPerRev = 2048;
  private String climber1State = "Initial Position";
  private final double GEAR_REDUCTION = 1.0 / 25.0;
  private final double SHAFT_DIAMETER_INCHES = 0.5; // Inches
  private boolean climber2IsDeployed = false; 

  /* 
    These are a constant offset to gravity. Set such that it retains a position of zero. This
    is an arbitrary output that is always added to the motor output.
  */ 
  private final double climber1Feedforward = 0;
  //private final double climber2Feedforward = 0;

  /* 
  Constants for extending climber
  25:1 gear ratio
  1/2" shaft
  */
  private final double MINIMUM_DISTANCE = 0;
  private final double MAXIMUM_DISTANCE = 100000; // UNITS INCHES
  private final double WINDOW_THRESHOLD = 1000; // UNITS INCHES
  private final double CLIMBER1_HEIGHT = 40000; // UNITS INCHES
  private final double REVERSE_DISTANCE_SETPOINT = 30000;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climberMotor1.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, 10);
    m_climberMotor1.setSelectedSensorPosition(0);
    m_climberMotor1.configForwardSoftLimitThreshold(convertPositionInchesToCounts(MAXIMUM_DISTANCE)); // Needs to be in counts
    m_climberMotor1.configForwardSoftLimitEnable(true);

    m_climberMotor1.configReverseSoftLimitThreshold(0);
    m_climberMotor1.configReverseSoftLimitEnable(true);

    m_climberMotor2.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, 10);
    m_climberMotor2.setSelectedSensorPosition(0);
  }

  private String monitorClimber1State(){
    // Using a string to represent the state of climber 1 is a hack implementation, but oh well.
    double encoderPostion = getPositionInches();
    if (getPositionInches() < MINIMUM_DISTANCE + WINDOW_THRESHOLD){
      return "Initial Position";
    }
    else if ((getPositionInches() > MINIMUM_DISTANCE + WINDOW_THRESHOLD) & (getPositionInches() < WINDOW_THRESHOLD + CLIMBER1_HEIGHT)){
      return "Raising To Climb";
    }
    else if ((getPositionInches() > CLIMBER1_HEIGHT - WINDOW_THRESHOLD) & (getPositionInches() < CLIMBER1_HEIGHT + WINDOW_THRESHOLD)){
      return "Ready to climb";
    }
    else if ((getPositionInches() > CLIMBER1_HEIGHT + WINDOW_THRESHOLD) & (getPositionInches() < MAXIMUM_DISTANCE - WINDOW_THRESHOLD)){
      return "Climbing";
    }
    else if ((getPositionInches() > MAXIMUM_DISTANCE - WINDOW_THRESHOLD) & (getPositionInches() < MAXIMUM_DISTANCE)){
      return "At Max Position";
    }
    else {
      return "Out of bounds";
    }

  }

  public String getClimber1State(){
    return climber1State;
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

  public void configureMotor1Feedback(){
    m_climberMotor1.configFactoryDefault();
		m_climberMotor1.setNeutralMode(NeutralMode.Brake);
    m_climberMotor1.setInverted(TalonFXInvertType.Clockwise);
		/* Config neutral deadband to be the smallest possible */
		m_climberMotor1.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
    m_climberMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.SHOOTER_PID_SLOT, Constants.kTimeoutMs);
											

		/* Config the peak and nominal outputs */
		m_climberMotor1.configNominalOutputForward(0, Constants.kTimeoutMs);
		m_climberMotor1.configNominalOutputReverse(0, Constants.kTimeoutMs);
		m_climberMotor1.configPeakOutputForward(1, Constants.kTimeoutMs);
		m_climberMotor1.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Config the position closed loop gains in slot0 */
		m_climberMotor1.config_kF(Constants.CLIMBER1_PID_SLOT, Constants.CLIMBER1_Kf, Constants.kTimeoutMs);
		m_climberMotor1.config_kP(Constants.CLIMBER1_PID_SLOT, Constants.CLIMBER1_Kp, Constants.kTimeoutMs);
		m_climberMotor1.config_kI(Constants.CLIMBER1_PID_SLOT, Constants.CLIMBER1_Ki, Constants.kTimeoutMs);
		m_climberMotor1.config_kD(Constants.CLIMBER1_PID_SLOT, Constants.CLIMBER1_Kd, Constants.kTimeoutMs);

    m_climberMotor1.set(ControlMode.Position, 0, DemandType.ArbitraryFeedForward, climber1Feedforward);
  }
  
  /*public void configureMotor2Feedback(){
    
    //m_climberMotor2.configFactoryDefault();
		//m_climberMotor2.setNeutralMode(NeutralMode.Brake);
    //m_climberMotor2.setInverted(TalonFXInvertType.Clockwise);
		//Config neutral deadband to be the smallest possible
		//m_climberMotor2.configNeutralDeadband(0.001);

		// Config sensor used for Primary PID [Velocity]
    //m_climberMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.SHOOTER_PID_SLOT, Constants.kTimeoutMs);
											

		// Config the peak and nominal outputs
		m_climberMotor2.configNominalOutputForward(0, Constants.kTimeoutMs);
		m_climberMotor2.configNominalOutputReverse(0, Constants.kTimeoutMs);
		m_climberMotor2.configPeakOutputForward(1, Constants.kTimeoutMs);
		m_climberMotor2.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		// Config the position closed loop gains in slot0
		m_climberMotor2.config_kF(Constants.CLIMBER2_PID_SLOT, Constants.CLIMBER2_Kf, Constants.kTimeoutMs);
		m_climberMotor2.config_kP(Constants.CLIMBER2_PID_SLOT, Constants.CLIMBER2_Kp, Constants.kTimeoutMs);
		m_climberMotor2.config_kI(Constants.CLIMBER2_PID_SLOT, Constants.CLIMBER2_Ki, Constants.kTimeoutMs);
		m_climberMotor2.config_kD(Constants.CLIMBER2_PID_SLOT, Constants.CLIMBER2_Kd, Constants.kTimeoutMs);

    m_climberMotor2.set(ControlMode.Position, 0, DemandType.ArbitraryFeedForward, climber2Feedforward);
    
  }*/

  @Override
  public void periodic() {
    climber1State = monitorClimber1State();
    // This method will be called once per scheduler run
    double currentPositionClimber1 = m_climberMotor1.getSelectedSensorPosition();
    SmartDashboard.putNumber("Climber 1 counts", currentPositionClimber1);
    SmartDashboard.putNumber("Climber 1 position (inches)", convertCountsToPositionInches(currentPositionClimber1));
    SmartDashboard.putNumber("Climber 1 error", m_climberMotor1.getClosedLoopError());

    double currentPositionClimber2 = m_climberMotor2.getSelectedSensorPosition();
    SmartDashboard.putNumber("Climber 2 counts", currentPositionClimber2);
    SmartDashboard.putNumber("Climber 2 position (inches)", convertCountsToPositionInches(currentPositionClimber2));
    //SmartDashboard.putNumber("Climber 2 error", m_climberMotor2.getClosedLoopError());

  }

  public double getPositionInches(){
    return convertCountsToPositionInches(m_climberMotor1.getSelectedSensorPosition());
  }
 
  public double convertCountsToPositionInches(double positionCounts){
    return positionCounts / encoderCountsPerRev * GEAR_REDUCTION * Math.PI * SHAFT_DIAMETER_INCHES;
  }

  public double convertPositionInchesToCounts(double positionInches){
    return positionInches * encoderCountsPerRev / (GEAR_REDUCTION * Math.PI * SHAFT_DIAMETER_INCHES);
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
