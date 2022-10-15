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
  private int timer; 
  private final PneumaticSubsystem m_pneumatics;
  
  public BackupToPosition(DifferentialDrivetrain drivetrain,
   PneumaticSubsystem pneumatics) {
    m_drivetrain = drivetrain;
    m_pneumatics = pneumatics;
    timer = 0; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pneumatics.turnOffCompressor();
    timer = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer += 20;
    if ((timer > 1) & (timer < 2500)) {
      m_drivetrain.driveForward(0.43);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pneumatics.enableCompressor();
    m_drivetrain.driveForward(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 2500; 
  }
}
