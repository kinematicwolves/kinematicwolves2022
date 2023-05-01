// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ControlDaCompressor;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.DriveRobotOpenLoop;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.SetDisabledState;
import frc.robot.commands.ShootTwoBalls;
import frc.robot.commands.ToggleSpeedLimit;
import frc.robot.commands.LightShowCommands.BlackLightshow;
import frc.robot.commands.LightShowCommands.BlueLightshow;
import frc.robot.commands.LightShowCommands.GayLighshow;
import frc.robot.commands.LightShowCommands.GreenLightshow;
import frc.robot.commands.LightShowCommands.PurpleLightshow;
import frc.robot.commands.LightShowCommands.RedLightshow;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final DifferentialDrivetrain m_drivetrainSubsystem = new DifferentialDrivetrain();
  private final PneumaticSubsystem m_pneumaticSubsystem = new PneumaticSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final HConveyorSubsystem m_hConveyorSubsystem = new HConveyorSubsystem();
  private final VConveyorSubsystem m_vConveyorSubsystem = new VConveyorSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final LightingSubsystem m_lighting = new LightingSubsystem();

  // Controllers
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER);

  // SmartDashboard chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    setDefaultCommands();

    m_chooser.setDefaultOption("Purple Led's", new PurpleLightshow(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    m_chooser.addOption("Red Led's", new RedLightshow(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    m_chooser.addOption("Green Led's", new GreenLightshow(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    m_chooser.addOption("Gay Led's", new GayLighshow(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    m_chooser.addOption("Blue Led's", new BlueLightshow(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    m_chooser.addOption("Black Led's", new BlackLightshow(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    SmartDashboard.putData(m_chooser);
  }
  // Sets Joystick drive controls as a deufault command
  private void setDefaultCommands(){
    m_drivetrainSubsystem.setDefaultCommand(new DriveRobotOpenLoop(m_drivetrainSubsystem, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver controls, dc = driver controller, mc = manipulator controller
    JoystickButton dc_aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    JoystickButton dc_bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    JoystickButton dc_xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value); 
    JoystickButton dc_yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    JoystickButton dc_rBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);

    //Driver Controller
    dc_aButton.whileHeld(new IntakeBalls(m_intakeSubsystem, m_hConveyorSubsystem, -1)); 
    dc_xButton.whenPressed(new DeployIntake(m_pneumaticSubsystem, m_intakeSubsystem));
    dc_yButton.whenPressed(new ControlDaCompressor(m_pneumaticSubsystem));
    dc_rBumper.whileHeld(new ShootTwoBalls(m_visionSubsystem, m_vConveyorSubsystem, 
                            m_hConveyorSubsystem, m_shooterSubsystem, m_intakeSubsystem, m_pneumaticSubsystem));
    dc_bButton.whenPressed(new ToggleSpeedLimit(m_drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new IntakeBalls(m_intakeSubsystem, m_hConveyorSubsystem, 0); 
    //this is just so auton does nothing and i forgot how to do the null thing
  }

    public Command getDisabledCommand(){
      Command disabled = new SetDisabledState(m_lighting, m_visionSubsystem);
      return disabled;
    } // Command to reset robot to initial state
    
    public Command getTeleopLightingCommand(){
      return m_chooser.getSelected(); 
    }
}
 