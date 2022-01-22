// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class DeployIntake extends CommandBase {
  private final PneumaticSubsystem m_pneumaticSubsystem;
  private final IntakeSubsystem m_intake;
  /** Creates a new DeployIntake. */
  public DeployIntake(PneumaticSubsystem pneumaticSubsystem, IntakeSubsystem intakeSubsystem) {
    m_pneumaticSubsystem = pneumaticSubsystem;
    m_intake = intakeSubsystem;
    addRequirements(m_pneumaticSubsystem, m_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake.isIntakeDeployed()){
      m_intake.setIntakeUndeployed(m_pneumaticSubsystem);
    }
    else{
      m_intake.setIntakeDeployed(m_pneumaticSubsystem);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
