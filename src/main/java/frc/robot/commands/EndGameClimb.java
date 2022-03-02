// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EndGameClimb extends SequentialCommandGroup {
  /** Creates a new EndGameClimb. */
  private final ClimberSubsystem m_climberSubsystem;
  public EndGameClimb(ClimberSubsystem climberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climberSubsystem = climberSubsystem;
    addCommands(
      new SetClimberToPosition(m_climberSubsystem, "end position", 0.4)
    );
  }
}
