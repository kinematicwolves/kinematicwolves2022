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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_climberMotor1 = new WPI_TalonFX(Constants.CLIMBER_MOTOR1);
  //private final WPI_TalonFX m_climberMotor2 = new WPI_TalonFX(Constants.CLIMBER_MOTOR2);
  public static Servo angleActuator_1 = new Servo(Constants.LINEAR_ACTUATOR_1); // PWM controlled
  private final int encoderCountsPerRev = 2048;
  private boolean climber1BrakeOn = false;
  private double maxServoExtention = 0.1; // meters
  private double distanceFromPivotPointMeters = 0.1; // Distance the servo is mounted from rotation point

  /* 
    These are a constant offset to gravity. Set such that it retains a position of zero. This
    is an arbitrary output that is always added to the motor output.
  */ 
  private final double climber1Feedforward = 0;
  private final double climber2Feedforward = 0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_climberMotor1.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, 10);
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
  
  public void configureMotor2Feedback(){
    /*
    //m_climberMotor2.configFactoryDefault();
		//m_climberMotor2.setNeutralMode(NeutralMode.Brake);
    //m_climberMotor2.setInverted(TalonFXInvertType.Clockwise);
		/* Config neutral deadband to be the smallest possible
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
    */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentPositionClimber1 = m_climberMotor1.getSelectedSensorPosition();
    SmartDashboard.putNumber("Climber 1 counts", currentPositionClimber1);
    SmartDashboard.putNumber("Climber 1 position (meters)", convertCountsToPosition(currentPositionClimber1));
    SmartDashboard.putNumber("Climber 1 error", m_climberMotor1.getClosedLoopError());
    SmartDashboard.putNumber("Motor current", m_climberMotor1.getStatorCurrent());
    SmartDashboard.putString("Climber state", getClimberState());

    // double currentPositionClimber2 = m_climberMotor1.getSelectedSensorPosition();
    // SmartDashboard.putNumber("Climber 2 counts", currentPositionClimber2);
    // SmartDashboard.putNumber("Climber 2 position (meters)", convertCountsToPosition(currentPositionClimber2));
    // SmartDashboard.putNumber("Climber 2 error", m_climberMotor2.getClosedLoopError());

  }

  public double getPositionMeters(){
    return convertCountsToPosition(m_climberMotor1.getSelectedSensorPosition());
  }

  public double convertCountsToPosition(double positionCounts){
    return 0.0;
  }

  public double convertPositionToCounts(double positionMeters){
    // (distance per count (rad))
    return 0.0;
  }

  public void setClimber1Position(double positionMeters){
    m_climberMotor1.set(ControlMode.Position, convertPositionToCounts(positionMeters));
  }

  public void setClimber2Position(double positionMeters){
    //m_climberMotor2.set(ControlMode.Position, convertPositionToCounts(positionMeters));
  }

  public double convertServoPositionToClimberAngleDegrees(double servoRawPosition){
    // Need to use some trig here and scale the raw servo position to an actual distance
    double extendedLengthMeters = servoRawPosition * maxServoExtention;
    
    return Math.atan(extendedLengthMeters / distanceFromPivotPointMeters);
  }

  public boolean servoAtPosition(double endRawPosition){
    // This moves so slow that PID control is not necessary
    // Position is a value between 0 and 1
    double currentRawPosition = angleActuator_1.getPosition();
    double error = Math.abs(currentRawPosition - endRawPosition);

    return (error < 0.02);
  }

  public void deployClimber1Brake(PneumaticSubsystem pneumaticSubsystem){
     climber1BrakeOn = true;
     pneumaticSubsystem.setClimberBrake();
  }

  public void releaseClimber1Brake(PneumaticSubsystem pneumaticSubsystem){
    climber1BrakeOn = false;
    pneumaticSubsystem.releaseClimberBrake();
  }

  public void setClimberMotor1Output(double commandedOutput){
    m_climberMotor1.set(commandedOutput);
  }

  public String getClimberState(){
    /*
    Adjust upperClimbWindowBound and lowerClimbWindowBound to be a range where the climber should stop 
    during climbing. Instead of tuning a complicated control loop, we will call this
    function to give us the current state of the climber, and use it to decide when 
    to stop it when it has reached its peak height for climb.
    */
    double upperClimbWindowBound = 1.0;
    double lowerClimbWindowBound = 0.95;
    double lowerThreshold = 0.1; // below this, if the limit switch is on, robot will be "at init position"
    double upperThreshold = 1.5; // Above this, if the limit switch is on, robot will be "at end position"

    /*
    Adjust the constants in the first two if/else if clauses so that
    there is a small point below which we know that the limit switch must 
    be in the init position, or above which it must be extended.
    */
    if  (getPositionMeters() < lowerThreshold){
      // Limit switch is active and position is not big, we must be at start spot
      return "init position";
    }
    else if ( getPositionMeters() > upperThreshold){
      // Limit switch is active and position is big, we must be at end spot
      return "end position";
    }
    else if ((getPositionMeters() < upperClimbWindowBound) & (getPositionMeters() > lowerClimbWindowBound)){
      return "ready to climb position";
    }
    else if (getPositionMeters() < lowerClimbWindowBound){
      return "Going up";
    }
    else if (getPositionMeters() > upperClimbWindowBound){
      return "Going down";
    }
    else {
      return "in between positions";
    }
  }

}