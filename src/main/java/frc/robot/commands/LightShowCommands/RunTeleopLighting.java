// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightShowCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RunTeleopLighting extends CommandBase {
  /** Creates a new RunTeleopLighting. */
  private final LightingSubsystem m_lighting;
  private final DifferentialDrivetrain m_drivetrain;
  private final VisionSubsystem m_vision;
  private Timer m_timer = new Timer();
  public RunTeleopLighting(LightingSubsystem lighting, DifferentialDrivetrain drivetrain, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_lighting = lighting;
    m_vision = vision;
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
    // I think this will turn orange after two minutes.
    if (m_timer.get() > 105){
      m_lighting.setTwinkleAnimation();
    }
    else {
      m_lighting.setPurpleSolidAnimation();
    }
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
