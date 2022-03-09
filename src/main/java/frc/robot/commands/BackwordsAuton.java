package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;


public class BackwordsAuton extends CommandBase {
  /**
   * Creates a new Backwords_Auton.
   */
  private final DifferentialDrivetrain drivetrain;
  double timer = 0;
  private final ShooterSubsystem m_shooterSubsystem;
  private final VConveyorSubsystem m_VConveyorSubsystem; 

  public BackwordsAuton(DifferentialDrivetrain drivetrain, ShooterSubsystem shooterSubsystem, VConveyorSubsystem vConveyorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
    this.m_VConveyorSubsystem = vConveyorSubsystem;
    addRequirements(m_VConveyorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drivetrain.moveBackward(-1 * Constants.AUTON_SPEED);
    // timer += 10;

    if(timer  < 2000)
        drivetrain.moveBackward(-1 * Constants.AUTON_SPEED);
    else if(timer >=2000 && timer <10000)
    {
      drivetrain.moveBackward(0);
      m_shooterSubsystem.setShooterMotorSpeed(4950);
      m_VConveyorSubsystem.runConveyorMotor(Constants.DEFAULT_VERTICAL_CONVEYOR_OUTPUT);
    }
    timer += 10;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return timer > 2000;  //yee haaa shoot
    return timer >= 10000;
  }
}
