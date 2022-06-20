// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LightShowCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RedAllianceLightshow extends CommandBase {
  private final LightingSubsystem m_LightingSubsystem; 
  private final PneumaticSubsystem pneumaticSubsystem; 
  private final VisionSubsystem visionSubsystem;
  private final DifferentialDrivetrain differentialDrivetrain; 
  private Timer m_timer = new Timer();

  /** Creates a new RedAllianceLightshow. */
  public RedAllianceLightshow(LightingSubsystem lightingSubsystem, PneumaticSubsystem pneumaticSubsystem, 
  VisionSubsystem visionSubsystem, DifferentialDrivetrain differentialDrivetrain) {
    m_LightingSubsystem = lightingSubsystem; 
    this.visionSubsystem = visionSubsystem;
    this.pneumaticSubsystem = pneumaticSubsystem; 
    this.differentialDrivetrain = differentialDrivetrain; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    visionSubsystem.turnLimelightOn();
    pneumaticSubsystem.enableCompressor();
    differentialDrivetrain.setHighGear();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() > 105){
      m_LightingSubsystem.setRedTwinkleAnimation();
    }
    else {
      m_LightingSubsystem.setPurpleTwinkleAnimation();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LightingSubsystem.setDisabledLightShow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
