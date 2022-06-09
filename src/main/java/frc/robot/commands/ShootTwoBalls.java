// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootTwoBalls extends CommandBase {
  private final ShooterSubsystem m_shooter;
  private final VisionSubsystem m_vision;
  private final IntakeSubsystem m_intake;
  private final PneumaticSubsystem m_pneumatics;
  private final ConveyorSubsystem m_conveyorSubsystem; 
  private int timer;
  /** Creates a new ShootTwoBalls. */
  public ShootTwoBalls(
    VisionSubsystem vision, ConveyorSubsystem conveyorSubsystem, 
    ShooterSubsystem shooter, IntakeSubsystem intake, PneumaticSubsystem pneumatics
  ) {
    m_shooter = shooter;
    m_conveyorSubsystem = conveyorSubsystem; 
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
    double shooterSpeedRPM = m_shooter.getMotorSpeedForDistance(distance);
    m_shooter.setShooterMotorSpeed(shooterSpeedRPM);

    timer += 20;
  
    if ((timer > 1) & (timer < 800)) {
      m_conveyorSubsystem.runHorizontalConveyor(0.7);//m_horizontal.runConveyorMotor(0.7);
      m_conveyorSubsystem.runVerticalConveyor(0.2);//m_verticalConeyor.runConveyorMotor(0.25);
    }
    if ((timer > 800) & (timer < 1400)){
      m_conveyorSubsystem.runVerticalConveyor(0.7);//m_verticalConeyor.runConveyorMotor(0.7);
    }
    if ((timer > 1400) & (timer < 2000)){
      m_conveyorSubsystem.runHorizontalConveyor(-1);//m_horizontal.runConveyorMotor(-1);
      m_intake.runIntakeMotor(-1);
      m_conveyorSubsystem.runVerticalConveyor(0);
    }
    if ((timer > 1400) & (timer < 2000)){
      //fix the timing in the sequence so the vertical conveyor doesnt reject the 2nd ball 
      m_conveyorSubsystem.runVerticalConveyor(0.7);
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
    return timer > 2000;
  }
}
