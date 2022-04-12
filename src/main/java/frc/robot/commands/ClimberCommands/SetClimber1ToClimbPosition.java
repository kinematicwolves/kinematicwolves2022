// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimber1ToClimbPosition extends CommandBase {
  private final ClimberSubsystem m_climbersubsystem; 
  private boolean safeForClimb;
  /** Creates a new SetClimber1ToClimbPosition. */
  public SetClimber1ToClimbPosition(ClimberSubsystem climberSubsystem) {
    m_climbersubsystem = climberSubsystem; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_climbersubsystem.isSafeForClimb()){
      m_climbersubsystem.setClimberMotor1Output(0.55);
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbersubsystem.setClimberMotor1Output(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_climbersubsystem.getClimber1State() == "Ready to climb") | (!m_climbersubsystem.isSafeForClimb());
  }
}
