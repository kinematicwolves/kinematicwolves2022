// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VConveyorSubsystem;


public class RunVerticalConveyor extends CommandBase {
 
  private final VConveyorSubsystem verticalconveyor;
  double commandedOutputFraction;

  /** Creates a new RunIntakeMotor. */
  public RunVerticalConveyor(VConveyorSubsystem vConveyorSubsystem, double commandedFraction) {
    this.verticalconveyor = vConveyorSubsystem; 
    this.commandedOutputFraction = commandedFraction;
    addRequirements(verticalconveyor);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    verticalconveyor.runConveyorMotor(commandedOutputFraction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    verticalconveyor.runConveyorMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}