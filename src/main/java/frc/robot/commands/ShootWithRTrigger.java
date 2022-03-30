// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
public class ShootWithRTrigger extends CommandBase {

private final ShooterSubsystem m_shooterSubsystem;
private final XboxController m_manipulatorController;
  /** Creates a new ShootWithTrigger. */
  public ShootWithRTrigger(ShooterSubsystem shooterSubsystem, XboxController manipulatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
    m_manipulatorController = manipulatorController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var triggerAxis = m_manipulatorController.getRightTriggerAxis();
    if ((triggerAxis > 0.005)) { //Set at a higher axis to give the shooter enough time to power up
      m_shooterSubsystem.setShooterMotorSpeed(1000); //RPM
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