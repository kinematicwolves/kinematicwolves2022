// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticSubsystem extends SubsystemBase {
  //private final PneumaticsControlModule pcm = new PneumaticsControlModule();
  private final PneumaticHub pHub = new PneumaticHub(9);
  private final DoubleSolenoid intakeSolenoid = pHub.makeDoubleSolenoid(Constants.INTAKE_SOLENOID_FWD, Constants.INTAKE_SOLENOID_RVS);
  private final DoubleSolenoid drivetrainSolenoid = pHub.makeDoubleSolenoid(Constants.DRVTRN_SOL_FWD_CHN, Constants.DRVTRN_SOL_RVS_CHN);
  private final DoubleSolenoid climberBrakeSolenoid = pHub.makeDoubleSolenoid(Constants.CLIMBER_BRAKE_FWD_CHN, Constants.CLIMBER_BRAKE_RVS_CHN);
  /** Creates a new PneumaticSubsystem. */



  public PneumaticSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeDeployed(){
    intakeSolenoid.set(Value.kReverse);
  }

  public void setIntakeUndeployed(){
    intakeSolenoid.set(Value.kForward);
  }

  public void setDrivetrainSolenoidFoward(){
    drivetrainSolenoid.set(Value.kReverse);
  }

  public void setDrivetrainSolenoidReverse(){
    drivetrainSolenoid.set(Value.kForward);
  }

  public void setClimberBrake(){
    climberBrakeSolenoid.set(Value.kForward);
  }

  public void releaseClimberBrake(){
    climberBrakeSolenoid.set(Value.kReverse);
  }
}
