// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightShowCommands;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RunTeleopLighting extends CommandBase {
  /** Creates a new RunTeleopLighting. */
  private final LightingSubsystem m_lighting;
  private final ShooterSubsystem m_shooterSubsystem; 
  private final DifferentialDrivetrain m_drivetrain;
  private final VisionSubsystem m_vision;
  private final PneumaticSubsystem m_pneumaticSubsystem;
  private Timer m_timer = new Timer();
  public RunTeleopLighting(LightingSubsystem lighting, DifferentialDrivetrain drivetrain, VisionSubsystem vision, ShooterSubsystem shooterSubsystem, PneumaticSubsystem pneumaticSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_lighting = lighting;
    m_vision = vision;
    m_shooterSubsystem = shooterSubsystem; 
    m_pneumaticSubsystem = pneumaticSubsystem; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_pneumaticSubsystem.turnOffCompressor();
   m_vision.turnLimelightOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // I think this will turn orange after two minutes.
   m_lighting.setPurpleTwinkleAnimation();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lighting.setDisabledLightShow();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
