// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DriveForwardAuton extends CommandBase {
  /** Creates a new DriveForwardAuton. */
  /*
NOTE: horizontal conveyor subsystem is actually the intake due to some mixups 
and not wanting to break existing code under a time crunch.
  */
  private final DifferentialDrivetrain m_drivetrain;
  private final double m_distance; 
  private final double m_speed;
  private final IntakeSubsystem m_intake;
  private final HConveyorSubsystem m_hconveyorsubsystem;
  public DriveForwardAuton(DifferentialDrivetrain drivetrain, double distanceInches,
    double speed, IntakeSubsystem intake, HConveyorSubsystem hConveyorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_distance = distanceInches;
    m_speed = speed;
    m_intake = intake;
    m_hconveyorsubsystem = hConveyorSubsystem; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_drivetrain.setMotorsBrake();
    m_intake.runIntakeMotor(-1); // really the intake
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveForward(m_speed);
    System.out.println("**Distance driven: " + m_drivetrain.getXDistanceDrivenInches() + " **");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_drivetrain.setMotorsCoast();
    m_intake.runIntakeMotor(0); // really the intake
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.getXDistanceDrivenInches() > m_distance;
  }
}
