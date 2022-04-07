// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimber1OpenLoop extends CommandBase {

  private final ClimberSubsystem climber1;
  double commandedOutputFraction;

  /** Creates a new Climber2. */
  public RunClimber1OpenLoop(ClimberSubsystem climberSubsystem, double commandedFraction) {
    climber1 = climberSubsystem;
    commandedOutputFraction = commandedFraction;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber1.setClimberMotor1Output(commandedOutputFraction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber1.setClimberMotor1Output(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
