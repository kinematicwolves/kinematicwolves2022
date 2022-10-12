// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;

public class ShootingSequenceEmergencyStop extends CommandBase {
  private final ShooterSubsystem m_shooter;
  private final VConveyorSubsystem m_verticalConeyor;
  private final HConveyorSubsystem m_horizontal;
  private final IntakeSubsystem m_intake;
  private final PneumaticSubsystem m_pneumatics;
  /** Creates a new ShootingSequenceEmergencyStop. */
  public ShootingSequenceEmergencyStop( ShooterSubsystem shooter, VConveyorSubsystem vconveyor, HConveyorSubsystem horizontal,
  IntakeSubsystem intake, PneumaticSubsystem pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_horizontal = horizontal;
    m_verticalConeyor = vconveyor;
    m_intake = intake;
    m_pneumatics = pneumatics;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_horizontal.runConveyorMotor(0);
    m_verticalConeyor.runConveyorMotor(0);
    m_shooter.setShooterMotorSpeed(0);
    m_pneumatics.enableCompressor();
    m_intake.runIntakeMotor(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
