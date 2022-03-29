// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LinearActuator;


public class DeploySecondArm extends CommandBase {
private final LinearActuator m_linearActuator;
  /* Creates a new DeploySecondArm. */
  public DeploySecondArm(LinearActuator linearActuator) {
    m_linearActuator = linearActuator;
    addRequirements(m_linearActuator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_linearActuator.setLinearActuatorPosition(.3); 

    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_linearActuator.servo_at_position(m_linearActuator);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
