// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class Climber1Timed extends CommandBase {

  private final ClimberSubsystem climber1;
  double commandedOutputFraction;



  /** Creates a new Climber2. */
  public Climber1Timed(ClimberSubsystem climberSubsystem, double output) {
    this.climber1 = climberSubsystem;
    this.commandedOutputFraction = output;
    addRequirements(climber1);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      climber1.setClimberMotor1Output(commandedOutputFraction);
    }
  

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
