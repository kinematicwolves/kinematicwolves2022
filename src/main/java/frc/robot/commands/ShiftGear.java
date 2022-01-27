// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.PneumaticSubsystem;

public class ShiftGear extends CommandBase {
  private final PneumaticSubsystem m_pneumaticSubsystem;
  private final DifferentialDrivetrain m_drivetrain; 
  /** Creates a new ShiftGear. */
  public ShiftGear(PneumaticSubsystem pneumaticSubsystem, DifferentialDrivetrain drivetrain) {
    m_pneumaticSubsystem = pneumaticSubsystem;
    m_drivetrain = drivetrain;
    addRequirements(m_pneumaticSubsystem, m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_drivetrain.isInHighGear()){
      m_drivetrain.shiftToLowGear(m_pneumaticSubsystem);
    }
    else{
      m_drivetrain.shiftToHighGear(m_pneumaticSubsystem);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
