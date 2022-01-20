
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.Constants;
import frc.robot.subsystems.DifferentialDrivetrain;
//import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.ConveyorSubsystem;
//import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Backup_Shoot_Auton extends SequentialCommandGroup {
  /**
   * Creates a new AutonLineUpShootBall.
   */
  public Backup_Shoot_Auton(DifferentialDrivetrain drivetrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new Backwords_Auton(drivetrain)
    );
  }
}
