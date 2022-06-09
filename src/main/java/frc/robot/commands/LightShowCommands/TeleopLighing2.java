//Yes there's a spelling error but i dont feel like fixing it 


package frc.robot.commands.LightShowCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TeleopLighing2 extends CommandBase {
  /** Creates a new RunTeleopLighting. */
  private final LightingSubsystem m_lighting;
  private final VisionSubsystem m_vision;
  private final PneumaticSubsystem m_pneumaticSubsystem;
  private Timer m_timer = new Timer();
  public TeleopLighing2(LightingSubsystem lighting, VisionSubsystem vision, PneumaticSubsystem pneumaticSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lighting = lighting;
    m_vision = vision;
    m_pneumaticSubsystem = pneumaticSubsystem; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_pneumaticSubsystem.turnOffCompressor();
   m_vision.turnLimelightOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // I think this will turn orange after two minutes.
   m_lighting.setRedTwinkleAnimation();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lighting.setDisabledLightShow();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
