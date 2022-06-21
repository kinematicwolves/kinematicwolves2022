// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class LeftPositionAuton1 extends CommandBase {
  private final DifferentialDrivetrain differentialDrivetrain; 
  private final HConveyorSubsystem hConveyorSubsystem; 
  private final VConveyorSubsystem vConveyorSubsystem; 
  private final IntakeSubsystem intakeSubsystem; 
  private final ShooterSubsystem shooterSubsystem; 
  private final VisionSubsystem visionSubsystem; 
  private final PneumaticSubsystem pneumaticSubsystem; 
  private int timer; 
  /** Creates a new RightPostionAuton1. */
  public LeftPositionAuton1(DifferentialDrivetrain differentialDrivetrain, HConveyorSubsystem hConveyorSubsystem, 
  VConveyorSubsystem vConveyorSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, 
  VisionSubsystem visionSubsystem, PneumaticSubsystem pneumaticSubsystem) {
    this.differentialDrivetrain = differentialDrivetrain; 
    this.hConveyorSubsystem = hConveyorSubsystem; 
    this.vConveyorSubsystem = vConveyorSubsystem;
    this.intakeSubsystem = intakeSubsystem; 
    this.shooterSubsystem = shooterSubsystem; 
    this.visionSubsystem = visionSubsystem; 
    this.pneumaticSubsystem = pneumaticSubsystem; 
    timer = 0; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    pneumaticSubsystem.turnOffCompressor();
    intakeSubsystem.setIntakeDeployed(pneumaticSubsystem);
    visionSubsystem.turnLimelightOn();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer += 20;

    if ((timer > 0) & (timer < 1000)) {
      differentialDrivetrain.driveForward(0.5);
      intakeSubsystem.runIntakeMotor(-1);
      hConveyorSubsystem.runConveyorMotor(-1);
    }
    if ((timer > 1000) & (timer < 3000)) {
      differentialDrivetrain.rotateDrivetrainToTarget(0.33, visionSubsystem);
    }
    if ((timer > 3000) & (timer < 5200)) {
      double distance = visionSubsystem.getFilteredDistance();
      double shooterSpeedRPM = shooterSubsystem.getMotorSpeedForDistance(distance);
      shooterSubsystem.setShooterMotorSpeed(shooterSpeedRPM);
    }
    if ((timer > 3000) & (timer < 4000)) {
      hConveyorSubsystem.runConveyorMotor(0.7);
      vConveyorSubsystem.runConveyorMotor(0.25);
    }
    if ((timer > 4000) & (timer < 5200)){
      vConveyorSubsystem.runConveyorMotor(0.7);
    }
    if ((timer > 4600) & (timer < 5200)){
      hConveyorSubsystem.runConveyorMotor(-1);
      intakeSubsystem.runIntakeMotor(-1);
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pneumaticSubsystem.enableCompressor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 5200;
  }
}
