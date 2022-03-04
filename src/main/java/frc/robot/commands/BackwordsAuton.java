package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;

public class BackwordsAuton extends CommandBase {
 // private final VConveyorSubsystem verticalconveyor;
  //private final ShooterSubsystem shootersubsystem;
 // private final DifferentialDrivetrain drivetrain;  
  /**
   * Creates a new Backwords_Auton.
   */
  private final DifferentialDrivetrain drivetrain;
  double timer = 0;

  public BackwordsAuton(DifferentialDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.moveBackward(-1 * Constants.AUTON_SPEED); 
    timer += 10; 
    }
  //(RunVerticalConveyor, Constants.DEFAULT_VERTICAL_CONVEYOR_OUTPUT);
  //}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 2000;
  }
}
