// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightShowCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SetDisabledState extends CommandBase {
  private final LightingSubsystem m_lighting;
  private final VisionSubsystem m_vision;
  /** Creates a new SetDisabledState. */
  public SetDisabledState(LightingSubsystem lighting, VisionSubsystem vision) {
    m_lighting = lighting;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lighting.setDisabledLightShow();
    m_vision.turnLimelightOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
