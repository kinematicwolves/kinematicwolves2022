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
    m_intake.setIntakeDeployed(m_pneumatics);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    timer += 20;

    if ((timer > 800) & (timer < 1000)) {
      m_horizontal.runConveyorMotor(1);
      //ball seperation in the indexer
    }
    if ((timer > 1000) & (timer < 1600)){
      m_verticalConeyor.runConveyorMotor(0.8);
      //lifts first ball to shooter fly wheel | shoots ball one
    }
    if ((timer > 1600) & (timer < 2300)) {
      m_verticalConeyor.runConveyorMotor(0);
      /*this stops the vertical conveyor to not put any oposing force to 
      the second ball traveling between the deadzone*/
    }
    if ((timer > 1700) & (timer < 2300)){
      m_horizontal.runConveyorMotor(-1);
      m_intake.runIntakeMotor(-1);
      //launch's second ball to the vertical conveyor 
    }
    if ((timer > 2300) & (timer < 3000)){
      m_horizontal.runConveyorMotor(-1);
      m_verticalConeyor.runConveyorMotor(0.8);
      //fires second ball
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_horizontal.runConveyorMotor(0);
    m_verticalConeyor.runConveyorMotor(0);
    m_shooter.setShooterMotorSpeed(0);
    m_pneumatics.enableCompressor();
    m_intake.runIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 3000;
  }
}
