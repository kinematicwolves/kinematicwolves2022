// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class AlignWithTarget extends CommandBase {
  /** Creates a new AlignWithTarget. */
  private final DifferentialDrivetrain m_drivetrain;
  private final VisionSubsystem m_visionSubsystem;
  private final double m_alignSpeed;
  public AlignWithTarget(VisionSubsystem visionSubsystem, DifferentialDrivetrain drivetrain, double alignSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_visionSubsystem = visionSubsystem;
    m_alignSpeed = alignSpeed;
    addRequirements(m_visionSubsystem, m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.rotateDrivetrainToTarget(m_alignSpeed, m_visionSubsystem);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.rotateDrivetrainToTarget(0, m_visionSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.isLinedUp(m_visionSubsystem);
  }
}
