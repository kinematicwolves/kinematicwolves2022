// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.PneumaticSubsystem;


public class BackupToPosition extends CommandBase {
  /** Creates a new BackupToPosition. */
  private final DifferentialDrivetrain m_drivetrain;
  private final double m_distance; 
  private final double m_speed;

  private final PneumaticSubsystem m_pneumatics;
  
  public BackupToPosition(DifferentialDrivetrain drivetrain, double distanceInches,
  double speed, PneumaticSubsystem pneumatics) {
    m_drivetrain = drivetrain;
    m_distance = distanceInches;
    m_speed = speed;
    m_pneumatics = pneumatics;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.driveForward(m_speed);
    m_pneumatics.turnOffCompressor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // NOTE: The robot should go backward until hits a distance
    m_drivetrain.driveForward(-1 * m_speed);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pneumatics.enableCompressor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drivetrain.getXDistanceDrivenInches()) > m_distance;
  }
}
