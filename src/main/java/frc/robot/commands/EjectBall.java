// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EjectBall extends CommandBase {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final ConveyorSubsystem conveyorSubsystem; 
  /** Creates a new EjectBall. */
  public EjectBall(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    m_ShooterSubsystem = shooterSubsystem; 
    this.conveyorSubsystem = conveyorSubsystem; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.setShooterMotorSpeed(1234);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyorSubsystem.runVerticalConveyor(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setShooterMotorSpeed(0);
    conveyorSubsystem.runVerticalConveyor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
