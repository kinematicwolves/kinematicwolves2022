// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootTwoBalls extends CommandBase {
  private final ShooterSubsystem m_shooter;
  private final VConveyorSubsystem m_verticalConeyor;
  private final VisionSubsystem m_vision;
  private final HConveyorSubsystem m_horizontal;
  private final IntakeSubsystem m_intake;
  private final PneumaticSubsystem m_pneumatics;
  private int timer;
  /** Creates a new ShootTwoBalls. */
  public ShootTwoBalls(
    VisionSubsystem vision, VConveyorSubsystem vconveyor, HConveyorSubsystem horizontal, 
    ShooterSubsystem shooter, IntakeSubsystem intake, PneumaticSubsystem pneumatics
  ) {
    m_shooter = shooter;
    m_horizontal = horizontal;
    m_verticalConeyor = vconveyor;
    m_vision = vision;
    m_intake = intake;
    m_pneumatics = pneumatics;
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
    m_intake.setIntakeDeployed(m_pneumatics);
    m_shooter.setShooterMotorSpeed(4200); // config in rpm

    timer += 20;
    if ((timer > 1500) & (timer < 2300)) {
      m_horizontal.runConveyorMotor(0.8);
      m_verticalConeyor.runConveyorMotor(-0.1);
    }
    if ((timer > 2350) & (timer < 4000)){
      m_verticalConeyor.runConveyorMotor(-8.5);
    }
    if ((timer > 3200) & (timer < 4000)){
      m_horizontal.runConveyorMotor(-1);
      m_intake.runIntakeMotor(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_horizontal.runConveyorMotor(0);
    m_verticalConeyor.runConveyorMotor(0);
    m_shooter.setShooterMotorSpeed(0);
    m_intake.runIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 4000;
  }
}
