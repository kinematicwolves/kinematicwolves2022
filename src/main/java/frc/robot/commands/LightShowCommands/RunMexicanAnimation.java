// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightShowCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightingSubsystem;

public class RunMexicanAnimation extends CommandBase {
  /** Creates a new MexicanAnimation. */
  private final LightingSubsystem m_lighting;
  public RunMexicanAnimation(LightingSubsystem lighting) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lighting = lighting; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lighting.setMexicanColorAnimation();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lighting.setGreenSolidAnimation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
