// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* The command should technically look like this but I have a 
 * feeling that somthing's off about it. 
 */

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimber1ToClimbPosition extends CommandBase {
  /** Creates a new RaiseClimber1. */
  private final ClimberSubsystem m_climber;
  public SetClimber1ToClimbPosition(ClimberSubsystem climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climber.isSafeForClimb()){
      m_climber.setClimberMotor1Output(Constants.DEFAULT_CLIMBER_OUTPUT); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setClimberMotor1Output(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //if this doesnt work then try line 49 instead 
    return (m_climber.getClimber1State() == "Ready to climb");
  }

}