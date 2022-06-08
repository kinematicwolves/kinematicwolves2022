// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightShowCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightingSubsystem;

public class BlueAllianceLightshow extends CommandBase {
  private final LightingSubsystem m_LightingSubsystem;
  private Timer m_timer = new Timer();

  /** Creates a new PurpleTwinkleLightShow. */
  public BlueAllianceLightshow(LightingSubsystem lightingSubsystem) {
    m_LightingSubsystem = lightingSubsystem; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() > 105){
      m_LightingSubsystem.setBlueTwinkleAnimation();
    }
    else {
      m_LightingSubsystem.setPurpleTwinkleAnimation();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
