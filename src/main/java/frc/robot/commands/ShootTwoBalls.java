// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootTwoBalls extends CommandBase {
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake; // Horizontal conveyor
  private final VConveyorSubsystem m_verticalConeyor;
  private final VisionSubsystem m_vision;
  private int timer;
  /** Creates a new ShootTwoBalls. */
  public ShootTwoBalls(
    VisionSubsystem vision, VConveyorSubsystem vconveyor, IntakeSubsystem intake, ShooterSubsystem shooter
  ) {
    m_shooter = shooter;
    m_intake = intake;
    m_verticalConeyor = vconveyor;
    m_vision = vision;
    timer = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = m_vision.getFilteredDistance();
    double shooterSpeedRPM = m_shooter.getMotorSpeedForDistance(distance);
    m_shooter.setShooterMotorSpeed(shooterSpeedRPM);

    timer += 20;
    if ((timer > 900) & (timer < 1100)){
      m_verticalConeyor.runConveyorMotor(0.80);
    }
    else if ((timer > 1300) & (timer < 5000)){
      m_intake.runIntakeMotor(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runIntakeMotor(0);
    m_verticalConeyor.runConveyorMotor(0);
    m_shooter.setShooterMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 3500;
  }
}
