// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;

public class ShootWithRTrigger extends CommandBase {

private final ShooterSubsystem m_shooterSubsystem;
private final XboxController m_manipulatorController;
private final VConveyorSubsystem m_VConveyorSubsystem;
private final HConveyorSubsystem m_hConveyorSubsystem;
double timer = 0;

  /** Creates a new ShootWithTrigger. */
  public ShootWithRTrigger(ShooterSubsystem shooterSubsystem, VConveyorSubsystem vConveyorSubsystem, HConveyorSubsystem hConveyorSubsystem, XboxController manipulatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
    this.m_VConveyorSubsystem = vConveyorSubsystem;
    addRequirements(m_VConveyorSubsystem);
    this.m_hConveyorSubsystem = hConveyorSubsystem;
    addRequirements(m_hConveyorSubsystem);
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
      if(timer  < 1000)
      m_shooterSubsystem.setShooterMotorSpeed(4500);
  else if(timer >=1000 && timer <10000)
  {
    m_VConveyorSubsystem.runConveyorMotor(Constants.DEFAULT_VERTICAL_CONVEYOR_OUTPUT);
    m_hConveyorSubsystem.runConveyorMotor(Constants.DEFAULT_HORIZONTAL_CONVEYOR_OUTPUT);
  }
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