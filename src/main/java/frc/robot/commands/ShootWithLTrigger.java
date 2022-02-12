// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithLTrigger extends CommandBase {

private final ShooterSubsystem m_shooter_subsystem;
private final ConveyorSubsystem m_conveyor_subsystem;
private final XboxController m_manipulatorController;

  /** Creates a new ShootWithTrigger. */
  public ShootWithLTrigger(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem, XboxController manipulatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter_subsystem = shooterSubsystem;
    addRequirements(m_shooter_subsystem);
    m_conveyor_subsystem = conveyorSubsystem;
    addRequirements(m_conveyor_subsystem);
    m_manipulatorController = manipulatorController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_manipulatorController.getLeftTriggerAxis() > 0.005) { //Set at a higher axis to give the shooter enough time to power up
      m_shooter_subsystem.setShooterMotorSpeed(5000); //RPM
      m_conveyor_subsystem.runConveyorMotor(0.9);
    }
    if (m_manipulatorController.getLeftTriggerAxis() > 1.0) {
      m_conveyor_subsystem.runConveyorMotor(0.9);
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
