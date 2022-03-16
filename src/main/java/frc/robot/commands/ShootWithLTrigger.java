// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootWithLTrigger extends CommandBase {

private final ShooterSubsystem m_shooterSubsystem;
private final XboxController m_manipulatorController;
private final VisionSubsystem m_visionSubsystem;
  /** Creates a new ShootWithTrigger. */
  public ShootWithLTrigger(ShooterSubsystem shooterSubsystem, XboxController manipulatorController, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_visionSubsystem = visionSubsystem;
    addRequirements(m_shooterSubsystem, m_visionSubsystem);
    m_manipulatorController = manipulatorController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var triggerAxis = m_manipulatorController.getLeftTriggerAxis();
    if ((triggerAxis > 0.005)) { //Set at a higher axis to give the shooter enough time to power up
      double targetDistanceInches = m_visionSubsystem.getFilteredDistance();
      double requiredShooterSpeedRPM = m_shooterSubsystem.getMotorSpeedForDistance(targetDistanceInches);
      m_shooterSubsystem.setShooterMotorSpeed(requiredShooterSpeedRPM); //RPM
      // m_shooterSubsystem.setShooterMotorSpeed(4930); // use the if not using calibration table
    }

    else {
      
      m_shooterSubsystem.setShooterMotorSpeed(0); 
    }
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