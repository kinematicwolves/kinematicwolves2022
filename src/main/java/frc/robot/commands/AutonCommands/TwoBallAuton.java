// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.ShootTwoBalls;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuton extends SequentialCommandGroup {
  /** Creates a new TwoBallAuton. */
  public TwoBallAuton(PneumaticSubsystem pneumatics, IntakeSubsystem intake, HConveyorSubsystem hConveyorSubsystem, 
    DifferentialDrivetrain drivetrain, VisionSubsystem vision, VConveyorSubsystem vconveyor, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeployIntake(pneumatics, intake),
      new DriveForwardAuton(drivetrain, 55, -0.5, intake, hConveyorSubsystem, pneumatics), // Drives forward while running intake
      new RotateToTarget(drivetrain, vision, -1 * 0.45), // Rotate until target found
      new ShootTwoBalls(vision, vconveyor, hConveyorSubsystem, shooter, intake, pneumatics)
    );
  }
}