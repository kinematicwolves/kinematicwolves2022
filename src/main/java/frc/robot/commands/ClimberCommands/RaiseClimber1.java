// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class RaiseClimber1 extends CommandBase {
  /** Creates a new RaiseClimber1. */
  private final ClimberSubsystem m_climber;
  private final IntakeSubsystem m_intake;
  private final PneumaticSubsystem m_pneumatics;
  public RaiseClimber1(ClimberSubsystem climber, IntakeSubsystem intake, PneumaticSubsystem pneumatics) {
    m_climber = climber;
    m_intake = intake;
    m_pneumatics = pneumatics;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_climber.isSafeForClimb()){
      m_climber.setClimberMotor1Output(Constants.DEFAULT_CLIMBER_OUTPUT);
      m_intake.setIntakeUndeployed(m_pneumatics);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setClimberMotor1Output(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!m_climber.isSafeForClimb()) | (m_climber.atClimb1Position());
  }
}
