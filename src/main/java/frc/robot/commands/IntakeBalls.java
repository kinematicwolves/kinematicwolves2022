// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBalls extends CommandBase {
 
  private final IntakeSubsystem intake;
  private final HConveyorSubsystem horizontal; 
  //private final PneumaticSubsystem pneumatics; 
  double commandedOutputFraction;

  /** Creates a new RunIntakeMotor. */
  public IntakeBalls(IntakeSubsystem intakeSubsystem, HConveyorSubsystem hConveyorSubsystem, double commandedFraction) {
    this.intake = intakeSubsystem; 
    horizontal = hConveyorSubsystem; 
    //pneumatics = pneumaticSubsystem; 
    this.commandedOutputFraction = commandedFraction;
    addRequirements(intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // intake.setIntakeDeployed(pneumatics);
    horizontal.runConveyorMotor(commandedOutputFraction);
    intake.runIntakeMotor(commandedOutputFraction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntakeMotor(0);
    horizontal.runConveyorMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
