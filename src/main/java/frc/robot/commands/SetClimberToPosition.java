// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimberToPosition extends CommandBase {
  private final ClimberSubsystem m_climberSubsystem;
  private String m_position;
  private double m_output; 
  /** Creates a new RaiseClimberToPosition. */
  public SetClimberToPosition(ClimberSubsystem climberSubsystem, String position, double output) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSubsystem = climberSubsystem;
    m_position = position;
    m_output = output;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_climberSubsystem.getClimberState() != m_position){
      m_climberSubsystem.setClimberMotor1Output(m_output);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.setClimberMotor1Output(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_climberSubsystem.getClimberState() == m_position) | (m_climberSubsystem.getClimberState() == "end position");
  }
}
