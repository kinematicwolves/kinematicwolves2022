// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final  WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(Constants.INTAKE_MOTOR);
  private boolean intakeIsDeployed = false; 
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntakeMotor(double requestedOutputFraction){
    m_intakeMotor.set(requestedOutputFraction);  
  }

  public boolean isIntakeDeployed(){
    return intakeIsDeployed; 
  }

  public void setIntakeDeployed(PneumaticSubsystem pneumaticSystem){
    // add solenoid as input and flip here
    intakeIsDeployed = true; 
    pneumaticSystem.setIntakeDeployed();
  }

  public void setIntakeUndeployed(PneumaticSubsystem pneumaticSubsystem){
    // Add solenoid as input and flip here
    intakeIsDeployed = false; 
    pneumaticSubsystem.setIntakeUndeployed();
  }
}
