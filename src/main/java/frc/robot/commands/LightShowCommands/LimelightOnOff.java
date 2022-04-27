// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightShowCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class LimelightOnOff extends CommandBase {
  private final VisionSubsystem m_visionSubsystem;
  /** Creates a new LimelightOnOff. */
  public LimelightOnOff(VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_visionSubsystem.isLimeLightOn()){
      m_visionSubsystem.turnLimelightOff();
    }
    else{
      m_visionSubsystem.turnLimelightOn();
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
