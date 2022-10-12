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
import frc.robot.commands.DeployIntake;
import frc.robot.commands.DriveRobotOpenLoop;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.ShiftGear;
import frc.robot.commands.AutonCommands.OneBallAuto;
import frc.robot.commands.AutonCommands.TwoBallAuton;
import frc.robot.commands.ClimberCommands.RunClimber1OpenLoop;
import frc.robot.commands.ClimberCommands.RunClimber2OpenLoop;
import frc.robot.commands.ClimberCommands.SetClimber1ToClimbPosition;
import frc.robot.commands.ClimberCommands.DeployClimber2;
import frc.robot.commands.LightShowCommands.BlueAllianceLightshow;
import frc.robot.commands.LightShowCommands.RedAllianceLightshow;
import frc.robot.commands.LightShowCommands.SetDisabledState;
import frc.robot.commands.ShooterCommands.EjectBall;
import frc.robot.commands.ShooterCommands.TwoBallShot;
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
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DifferentialDrivetrain drivetrain = new DifferentialDrivetrain();
  private final PneumaticSubsystem pneumaticSubsystem = new PneumaticSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final HConveyorSubsystem hConveyorSubsystem = new HConveyorSubsystem();
  private final VConveyorSubsystem vConveyorSubsystem = new VConveyorSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final LightingSubsystem lightingSubsystem = new LightingSubsystem();
  // Controllers
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER);
  private final XboxController m_manipulatorController = new XboxController(Constants.MANIPULATOR_CONTROLLER);
  SendableChooser<Command> m_AutonChooser = new SendableChooser<>();
  SendableChooser<Command> m_LightsChooser = new SendableChooser<>(); 
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    setDefaultCommands();
    CameraServer.startAutomaticCapture();

     // A chooser for autonomous commands
     m_AutonChooser.setDefaultOption("1 Ball Auto", new OneBallAuto(drivetrain, pneumaticSubsystem, intakeSubsystem, visionSubsystem, lightingSubsystem, hConveyorSubsystem, vConveyorSubsystem, shooterSubsystem));
     m_AutonChooser.addOption("2 Ball Auto", new TwoBallAuton(pneumaticSubsystem, intakeSubsystem, hConveyorSubsystem, drivetrain, visionSubsystem, vConveyorSubsystem, shooterSubsystem));
     SmartDashboard.putData(m_AutonChooser);

     // A chooser for Lightshow commands
     m_LightsChooser.setDefaultOption("Red Alliance Lighshow", new RedAllianceLightshow(lightingSubsystem, visionSubsystem, drivetrain, pneumaticSubsystem));
     m_LightsChooser.addOption("Blue Alliance Lightshow", new BlueAllianceLightshow(lightingSubsystem, visionSubsystem, drivetrain, pneumaticSubsystem));
     SmartDashboard.putData(m_LightsChooser);
 }

  private void setDefaultCommands(){
    drivetrain.setDefaultCommand(new DriveRobotOpenLoop(drivetrain, m_driverController));
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
    JoystickButton dc_aButton = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    JoystickButton dc_bButton = new JoystickButton(m_driverController, XboxController.Button.kB.value);
    JoystickButton dc_yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    JoystickButton dc_xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value); 
    JoystickButton dc_lButton = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    JoystickButton dc_rJoystickButton = new JoystickButton(m_driverController, XboxController.Button.kRightStick.value);

    JoystickButton mc_aButton = new JoystickButton(m_manipulatorController, XboxController.Button.kA.value);
    JoystickButton mc_rButton = new JoystickButton(m_manipulatorController, XboxController.Button.kRightBumper.value);
    JoystickButton mc_lButton = new JoystickButton(m_manipulatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton mc_xButton = new JoystickButton(m_manipulatorController, XboxController.Button.kX.value);
    JoystickButton mc_bButton = new JoystickButton(m_manipulatorController, XboxController.Button.kB.value);
    JoystickButton mc_yButton = new JoystickButton(m_manipulatorController, XboxController.Button.kY.value);



  //Driver Controller
  dc_aButton.whenPressed(new ShiftGear(pneumaticSubsystem, drivetrain)); // A = Shift gear
  dc_yButton.whenPressed(new DeployClimber2(pneumaticSubsystem, climberSubsystem)); // Y = Deploy 2nd climber
  dc_xButton.whileHeld(new RunClimber1OpenLoop(climberSubsystem, 0.45)); // X = Run Climber 1 at 45%
  dc_bButton.whileHeld(new RunClimber2OpenLoop(climberSubsystem, 0.45)); // B = Run Climber 2 at 45%

  dc_rButton.whileHeld(new TwoBallShot(drivetrain, pneumaticSubsystem, intakeSubsystem, 
    visionSubsystem, lightingSubsystem, hConveyorSubsystem, vConveyorSubsystem, shooterSubsystem)); // RB  = Line up & Shoot 2 balls (Untested)
  

  //Munipulator Controller 
  mc_aButton.whileHeld(new IntakeBalls(intakeSubsystem, hConveyorSubsystem, -1)); // A = Intake
  mc_yButton.whileHeld(new IntakeBalls(intakeSubsystem, hConveyorSubsystem, 1)); // Y = Reverse Intake
  mc_xButton.whenPressed(new DeployIntake(pneumaticSubsystem, intakeSubsystem)); // X = Deploy / Undeploy intake
  mc_bButton.whileHeld(new EjectBall(shooterSubsystem, vConveyorSubsystem)); // B = Eject ball from vertical 

  mc_lButton.whenPressed(new SetClimber1ToClimbPosition(climberSubsystem)); // LB = Set climber 1 to climb position (Untested)

  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Auton selector on smart dashboard
    return m_AutonChooser.getSelected(); 
  }

    public Command getDisabledCommand(){
      // Command to reset robot to initial state
      Command disabled = new SetDisabledState(lightingSubsystem, visionSubsystem);
      return disabled;
    }
    
    public Command getTeleopLightingCommand(){
    // Alliance color selector for leds
      return m_LightsChooser.getSelected();
    }
}

 