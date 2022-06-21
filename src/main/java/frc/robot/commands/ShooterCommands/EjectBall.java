// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;

public class EjectBall extends CommandBase {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final VConveyorSubsystem m_VConveyorSubsystem; 
  /** Creates a new EjectBall. */
  public EjectBall(ShooterSubsystem shooterSubsystem, VConveyorSubsystem vConveyorSubsystem) {
    m_ShooterSubsystem = shooterSubsystem; 
    m_VConveyorSubsystem = vConveyorSubsystem; 
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
    m_VConveyorSubsystem.runConveyorMotor(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setShooterMotorSpeed(0);
    m_VConveyorSubsystem.runConveyorMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
