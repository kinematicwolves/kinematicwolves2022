
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Constants;
import frc.robot.subsystems.DifferentialDrivetrain;
//import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.ConveyorSubsystem;
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class BackupShootAuton extends SequentialCommandGroup {
  /**
   * Creates a new AutonLineUpShootBall.
   */
  public BackupShootAuton(DifferentialDrivetrain drivetrain, 
  HConveyorSubsystem h_conveyor, VConveyorSubsystem v_conveyor,
  ShooterSubsystem shooterSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new BackwordsAuton(drivetrain),
      new AutonShootBall(shooterSubsystem, h_conveyor, v_conveyor)
    );
  }
}
