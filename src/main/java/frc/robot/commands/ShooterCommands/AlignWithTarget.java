// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignWithTarget extends CommandBase {
  /** Creates a new AlignWithTarget. */
  private final DifferentialDrivetrain m_drivetrain;
  private final VisionSubsystem m_visionSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem; 
  private final PneumaticSubsystem m_PneumaticSubsystem; 
  private final double m_alignSpeed;
  private final IntakeSubsystem intakeSubsystem; 
  public AlignWithTarget(VisionSubsystem visionSubsystem, DifferentialDrivetrain drivetrain, ShooterSubsystem shooterSubsystem, 
   PneumaticSubsystem pneumaticSubsystem, IntakeSubsystem intakeSubsystem, double alignSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_visionSubsystem = visionSubsystem;
    m_alignSpeed = alignSpeed;
    m_ShooterSubsystem = shooterSubsystem; 
    m_PneumaticSubsystem = pneumaticSubsystem; 
    this.intakeSubsystem = intakeSubsystem; 
    addRequirements(m_visionSubsystem, m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // m_ShooterSubsystem.setShooterMotorSpeed(5100);//Starting the shooter during lineup will conserve battery life while shortening cycle time :)
    m_PneumaticSubsystem.turnOffCompressor();//This will conserve battery power so the shooter sequence isn't affected by low battery
    m_drivetrain.setLowGear(); // A little more power for anyone coming at us during lineup 
    intakeSubsystem.setIntakeDeployed(m_PneumaticSubsystem);
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.rotateDrivetrainToTarget(m_alignSpeed, m_visionSubsystem);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.rotateDrivetrainToTarget(0, m_visionSubsystem);
    m_drivetrain.setHighGear();
    double distance = m_visionSubsystem.getFilteredDistance();
    double shooterSpeedRPM = m_ShooterSubsystem.getMotorSpeedForDistance(distance);
    m_ShooterSubsystem.setShooterMotorSpeed(shooterSpeedRPM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.isLinedUp(m_visionSubsystem);
  }
}
