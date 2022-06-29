// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.ShooterCommands.AlignWithTarget;
import frc.robot.commands.ShooterCommands.ShootTwoBalls;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallAuto extends SequentialCommandGroup {
  /** Creates a new BackupShootBackup. */
  public OneBallAuto(DifferentialDrivetrain drivetrain, PneumaticSubsystem pneumatics, 
    IntakeSubsystem intake, VisionSubsystem vision, LightingSubsystem lighting, HConveyorSubsystem horizontal, 
    VConveyorSubsystem vconveyor, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /*
    The distances in BackupToPosition are abosolute here, so no need to worry about negative distances.
    */
    addCommands(
      new DeployIntake(pneumatics, intake),
      new BackupToPosition(drivetrain, 45, -0.5, pneumatics),
      new AlignWithTarget(vision, drivetrain, shooter, pneumatics, intake, 0.34),
      new ShootTwoBalls(vision, vconveyor, horizontal, shooter, intake, pneumatics),
      new BackupToPosition(drivetrain, 60, -0.5, pneumatics)
    );
  }
}
