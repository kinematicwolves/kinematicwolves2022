// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class ReverseIntake extends CommandBase {
 
  private final IntakeSubsystem intake;
  private final ConveyorSubsystem conveyorSubsystem; 
  //private final PneumaticSubsystem pneumatics; 

  /** Creates a new RunIntakeMotor. */
  public ReverseIntake(IntakeSubsystem intakeSubsystem, ConveyorSubsystem conveyorSubsystem) {
    this.intake = intakeSubsystem; 
    this.conveyorSubsystem = conveyorSubsystem; 
    //pneumatics = pneumaticSubsystem; 
    addRequirements(intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyorSubsystem.runHorizontalConveyor(1);
    intake.runIntakeMotor(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntakeMotor(0);
    conveyorSubsystem.runHorizontalConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}