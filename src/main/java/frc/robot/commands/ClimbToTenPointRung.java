// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class ClimbToTenPointRung extends CommandBase {
  /** Creates a new ClimbToTenPointRung. */
  private final ClimberSubsystem m_climber;
  private final PneumaticSubsystem m_pneumatics;
  private Timer m_timer = new Timer();

  public ClimbToTenPointRung(ClimberSubsystem climber, PneumaticSubsystem pneumatics) {
    m_climber = climber;
    m_pneumatics = pneumatics;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    if (m_climber.isSafeForSecondClimb()){
      m_climber.setClimber2Deployed(m_pneumatics);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() > 1.5){
      m_climber.setClimberMotor2Output(0.6);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setClimberMotor2Output(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_climber.isSafeForSecondClimb() | m_climber.isClimber2Finished();
  }
}
