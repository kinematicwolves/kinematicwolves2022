// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimber2OpenLoop extends CommandBase {

  private final ClimberSubsystem climber2;
  double commandedOutputFraction;
  private final XboxController m_manipulatorController;

  /** Creates a new Climber2. */
  public RunClimber2OpenLoop(ClimberSubsystem climberSubsystem, XboxController manipulatorController) {
    this.climber2 = climberSubsystem;
    m_manipulatorController = manipulatorController;
    addRequirements(climber2);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_manipulatorController.getPOV() == 0){
      climber2.setClimberMotor2Output(0.55);
    }
    else {
      climber2.setClimberMotor2Output(0);
    }
    if (m_manipulatorController.getPOV() == 180){
    climber2.setClimberMotor2Output(0.2);
    }
    else {
    climber2.setClimberMotor2Output(0);
  }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber2.setClimberMotor1Output(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}