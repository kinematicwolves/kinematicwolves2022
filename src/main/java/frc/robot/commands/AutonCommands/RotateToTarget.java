// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class RotateToTarget extends CommandBase {
  /** Creates a new RotateToTarget. */
  private final DifferentialDrivetrain m_drivetrain;
  private final VisionSubsystem m_vision;
  private final double m_speed;

  public RotateToTarget(DifferentialDrivetrain drivetrain, 
  VisionSubsystem vision, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_drivetrain.getGyroYAxis()) < 168)
    m_drivetrain.rotateClockwise(m_speed);
    else {
      m_drivetrain.rotateDrivetrainToTarget(0.31, m_vision);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_drivetrain.isLinedUp(m_vision)) & (Math.abs(m_drivetrain.getGyroYAxis()) > 150);
  }
}
