// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_horizontalConveyorMotor = new WPI_TalonFX(Constants.HORIZONTALCONVEYORMOTOR);
  private final WPI_TalonFX m_verticalConveyorMotor = new WPI_TalonFX(Constants.VERTICALCONVEYORMOTOR);
  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runConveyorMotor(double inputCommand){
    m_horizontalConveyorMotor.set(inputCommand);
    m_verticalConveyorMotor.set(inputCommand);
  }
}
