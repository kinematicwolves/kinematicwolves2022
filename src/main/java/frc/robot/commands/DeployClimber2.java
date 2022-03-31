// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;import frc.robot.subsystems.ClimberSubsystem;import frc.robot.subsystems.PneumaticSubsystem;

public class DeployClimber2 extends CommandBase {
  private final PneumaticSubsystem m_pneumaticSubsystem;
  private final ClimberSubsystem m_climberSubsystem; 
  /** Creates a new DeployIntake. */
  public DeployClimber2(PneumaticSubsystem pneumaticSubsystem, ClimberSubsystem climberSubsystem) {
    m_pneumaticSubsystem = pneumaticSubsystem;
    m_climberSubsystem = climberSubsystem;
    addRequirements(m_pneumaticSubsystem, climberSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_climberSubsystem.isClimber2Deployed()){
      m_climberSubsystem.setClimber2Undeployed(m_pneumaticSubsystem);
    }
    else{
      m_climberSubsystem.setClimber2Deployed(m_pneumaticSubsystem);
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
    return true;
  }
}
