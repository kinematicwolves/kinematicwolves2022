// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightShowCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightingSubsystem;

public class dcrColorAnimation extends CommandBase {
  private final LightingSubsystem m_lightingSubsystem;
  private int timer; 
  /** Creates a new dcrColorAnimation. */
  public dcrColorAnimation(LightingSubsystem lightingSubsystem) {
    m_lightingSubsystem = lightingSubsystem;
    timer = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer += 20;
    if ((timer > 0) & (timer < 1000)){
      m_lightingSubsystem.setGreenSolidAnimation();
    }
    if ((timer > 1000) & (timer < 2000)){
      m_lightingSubsystem.runWhiteAnimation();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lightingSubsystem.setRainbowAnimation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 2000;
  }
}