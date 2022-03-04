// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;

public class AutonShootBall extends CommandBase {
  private final ShooterSubsystem m_shooterSubsystem;
  private final HConveyorSubsystem m_hConveyorSubsystem;
  private final VConveyorSubsystem m_vConveyorSubsystem;
  private double m_timer;

  /** Creates a new AutonShootBall. */
  public AutonShootBall(ShooterSubsystem shooterSubsystem, HConveyorSubsystem h_conveyor, VConveyorSubsystem v_conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_hConveyorSubsystem = h_conveyor;
    m_vConveyorSubsystem = v_conveyor;
    m_timer = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setShooterMotorSpeed(5300);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_timer += 20;
    if (m_timer > 500){
      m_vConveyorSubsystem.runConveyorMotor(Constants.DEFAULT_HORIZONTAL_CONVEYOR_OUTPUT);
      m_hConveyorSubsystem.runConveyorMotor(Constants.DEFAULT_VERTICAL_CONVEYOR_OUTPUT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vConveyorSubsystem.runConveyorMotor(Constants.DEFAULT_HORIZONTAL_CONVEYOR_OUTPUT);
    m_hConveyorSubsystem.runConveyorMotor(Constants.DEFAULT_VERTICAL_CONVEYOR_OUTPUT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer > 4000;
  }
}
