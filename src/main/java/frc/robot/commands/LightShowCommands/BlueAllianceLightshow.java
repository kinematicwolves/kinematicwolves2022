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

public class BlueAllianceLightshow extends CommandBase {
  private final LightingSubsystem m_LightingSubsystem;
  private final VisionSubsystem visionSubsystem; 
  private final DifferentialDrivetrain differentialDrivetrain; 
  private final PneumaticSubsystem pneumaticSubsystem; 

  private Timer m_timer = new Timer();

  /** Creates a new PurpleTwinkleLightShow. */
  public BlueAllianceLightshow(LightingSubsystem lightingSubsystem, VisionSubsystem visionSubsystem, 
  DifferentialDrivetrain differentialDrivetrain, PneumaticSubsystem pneumaticSubsystem) {
    m_LightingSubsystem = lightingSubsystem; 
    this.visionSubsystem = visionSubsystem; 
    this.differentialDrivetrain = differentialDrivetrain; 
    this.pneumaticSubsystem = pneumaticSubsystem; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    visionSubsystem.turnLimelightOn();
    differentialDrivetrain.setHighGear();
    pneumaticSubsystem.enableCompressor();
    differentialDrivetrain.enableSpeedLimit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() > 105){
      m_LightingSubsystem.setPurpleTwinkleAnimation();
    }
    else {
      m_LightingSubsystem.setBlueTwinkleAnimation();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LightingSubsystem.setDisabledLightShow();
    visionSubsystem.turnLimelightOff();
    pneumaticSubsystem.turnOffCompressor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
