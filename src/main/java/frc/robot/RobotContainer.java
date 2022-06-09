// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
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
import frc.robot.commands.RunVerticalConveyor;
import frc.robot.commands.SetDisabledState;
import frc.robot.commands.SetShooterToSpeed;
import frc.robot.commands.ShiftGear;
import frc.robot.commands.ShootTwoBalls;
import frc.robot.commands.ToggleSpeedLimit;
import frc.robot.commands.ClimberCommands.DeployClimber2;
import frc.robot.commands.LightShowCommands.BlueLightshow;
import frc.robot.commands.LightShowCommands.GayTeleopLighshow;
import frc.robot.commands.LightShowCommands.LimelightOnOff;
import frc.robot.commands.LightShowCommands.TeleopLighing2;
import frc.robot.commands.LightShowCommands.TeleopLighting1;
import frc.robot.subsystems.ClimberSubsystem;
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
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final DifferentialDrivetrain m_drivetrainSubsystem = new DifferentialDrivetrain();
  private final PneumaticSubsystem m_pneumaticSubsystem = new PneumaticSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final HConveyorSubsystem m_hConveyorSubsystem = new HConveyorSubsystem();
  private final VConveyorSubsystem m_vConveyorSubsystem = new VConveyorSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final LightingSubsystem m_lighting = new LightingSubsystem();
  // Controllers
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER);
  private final XboxController m_manipulatorController = new XboxController(Constants.MANIPULATOR_CONTROLLER);
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    setDefaultCommands();
    CameraServer.startAutomaticCapture();

     // A chooser for autonomous commands
    
    m_chooser.setDefaultOption("Purple Led's", new TeleopLighting1(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    m_chooser.addOption("Red Led's", new TeleopLighing2(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    m_chooser.addOption("Gay Led's", new GayTeleopLighshow(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    m_chooser.addOption("Blue Led's", new BlueLightshow(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    m_chooser.addOption("Incase any one is light sensitive", new BlueLightshow(m_lighting, m_visionSubsystem, m_pneumaticSubsystem));
    SmartDashboard.putData(m_chooser);
  }

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
    JoystickButton dc_rButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    JoystickButton dc_lButton = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value); 
    JoystickButton dc_aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    JoystickButton dc_bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    JoystickButton dc_yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    JoystickButton dc_xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value); 
    JoystickButton dc_rJoystickButton = new JoystickButton(m_driverController, XboxController.Button.kRightStick.value);
    JoystickButton dc_lJoystickButton = new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value);

    JoystickButton mc_aButton = new JoystickButton(m_manipulatorController, XboxController.Button.kA.value);
    JoystickButton mc_rButton = new JoystickButton(m_manipulatorController, XboxController.Button.kRightBumper.value);
    JoystickButton mc_lButton = new JoystickButton(m_manipulatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton mc_xButton = new JoystickButton(m_manipulatorController, XboxController.Button.kX.value);
    JoystickButton mc_yButton = new JoystickButton(m_manipulatorController, XboxController.Button.kY.value);
    JoystickButton mc_bButton = new JoystickButton(m_manipulatorController, XboxController.Button.kB.value);
    JoystickButton mc_rJoystickButton = new JoystickButton(m_manipulatorController, XboxController.Button.kRightStick.value); 


  //Driver Controller
  dc_aButton.whenPressed(new ShiftGear(m_pneumaticSubsystem, m_drivetrainSubsystem)); 
  dc_yButton.whenPressed(new DeployClimber2(m_pneumaticSubsystem, m_climberSubsystem)); 
  dc_rJoystickButton.whenPressed(new ToggleSpeedLimit(m_drivetrainSubsystem));
  dc_bButton.whenPressed(new LimelightOnOff(m_visionSubsystem)); 
  dc_xButton.whenPressed(new ControlDaCompressor(m_pneumaticSubsystem)); //let me know if this works cause tbh it shouldn't but idk

  //Driver has both limelight and compressor controls
  //Have jose read out the buttons before driving



    //Munipulator Controller 
    mc_aButton.whileHeld(new IntakeBalls(m_intakeSubsystem, m_hConveyorSubsystem, -1)); 
    mc_xButton.whenPressed(new DeployIntake(m_pneumaticSubsystem, m_intakeSubsystem));
    mc_rButton.whileHeld(new ShootTwoBalls(m_visionSubsystem, m_vConveyorSubsystem, m_hConveyorSubsystem,
     m_shooterSubsystem, m_intakeSubsystem, m_pneumaticSubsystem)); 

    mc_lButton.whileHeld(new SetShooterToSpeed(m_shooterSubsystem, 1234));
    mc_lButton.whileHeld(new RunVerticalConveyor(m_vConveyorSubsystem, 0.8));
      //this is to eject balls
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
 