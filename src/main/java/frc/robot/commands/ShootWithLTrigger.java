// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;

public class ShootWithLTrigger extends CommandBase {

private final ShooterSubsystem m_shooterSubsystem;
private final VConveyorSubsystem m_vConveyorSubsystem;
private final XboxController m_manipulatorController;

  /** Creates a new ShootWithTrigger. */
  public ShootWithLTrigger(ShooterSubsystem shooterSubsystem, VConveyorSubsystem vConveyorSubsystem, XboxController manipulatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
    m_vConveyorSubsystem = vConveyorSubsystem;
    addRequirements(m_vConveyorSubsystem);
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
    if (triggerAxis > 0.5)
    m_shooterSubsystem.setShooterMotorSpeed(4500); //RPM
    else
    m_shooterSubsystem.setShooterMotorSpeed(0); //RPM
    /*if ((triggerAxis > 0.005) & (triggerAxis < 0.5)) { //Set at a higher axis to give the shooter enough time to power up
      m_shooterSubsystem.setShooterMotorSpeed(4500); //RPM
      
    }
    else if (triggerAxis >= 0.9){
      m_vConveyorSubsystem.runConveyorMotor(0.9);
    }
    else {
      m_vConveyorSubsystem.runConveyorMotor(0);
      m_shooterSubsystem.setShooterMotorSpeed(0);
    }*/
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