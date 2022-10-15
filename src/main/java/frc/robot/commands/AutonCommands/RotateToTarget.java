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
  private final double m_speed;
  private int timer;

  public RotateToTarget(DifferentialDrivetrain drivetrain, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_speed = speed;
    timer = 0; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer =+ 20;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((timer > 1) & (timer < 2500)) {
      m_drivetrain.rotateClockwise(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.rotateClockwise(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 2500;
  }
}
