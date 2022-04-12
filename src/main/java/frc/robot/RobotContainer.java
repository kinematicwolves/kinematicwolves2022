// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignWithTarget;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.DriveRobotOpenLoop;
import frc.robot.commands.RunHorizontalConveyor;
import frc.robot.commands.RunIntakeMotor;
import frc.robot.commands.RunVerticalConveyor;
import frc.robot.commands.SetDisabledState;
import frc.robot.commands.SetShooterToSpeed;
import frc.robot.commands.ShiftGear;
import frc.robot.commands.ShootTwoBalls;
import frc.robot.commands.AutonCommands.TwoBallAuton;
import frc.robot.commands.LightShowCommands.RunTeleopLighting;
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
  // private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final HConveyorSubsystem m_hConveyorSubsystem = new HConveyorSubsystem();
  private final VConveyorSubsystem m_vConveyorSubsystem = new VConveyorSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final LightingSubsystem m_lighting = new LightingSubsystem();
  // Controllers
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER);
  private final XboxController m_manipulatorController = new XboxController(Constants.MANIPULATOR_CONTROLLER);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    setDefaultCommands();
    CameraServer.startAutomaticCapture();
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

    JoystickButton mc_aButton = new JoystickButton(m_manipulatorController, XboxController.Button.kA.value);
    JoystickButton mc_rButton = new JoystickButton(m_manipulatorController, XboxController.Button.kRightBumper.value);
    JoystickButton mc_lButton = new JoystickButton(m_manipulatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton mc_xButton = new JoystickButton(m_manipulatorController, XboxController.Button.kX.value);
    JoystickButton mc_yButton = new JoystickButton(m_manipulatorController, XboxController.Button.kY.value);
    JoystickButton mc_bButton = new JoystickButton(m_manipulatorController, XboxController.Button.kB.value);


  //Driver Controller
    dc_lButton.whenPressed(new ShiftGear(m_pneumaticSubsystem, m_drivetrainSubsystem));
    dc_rButton.whileHeld(new AlignWithTarget(m_visionSubsystem, m_drivetrainSubsystem, 0.31));

  //Munipulator Controller 
  //-RunIntakeMotor = Horizontal Conveyor
  //-RunHorizontalConveyor = Intake Motor
  //Beware of polarities
    mc_aButton.whileHeld(new RunIntakeMotor(m_intakeSubsystem, -1));
    mc_aButton.whileHeld(new RunHorizontalConveyor(m_hConveyorSubsystem, -1));
    mc_aButton.whileHeld(new RunVerticalConveyor(m_vConveyorSubsystem, 0.1));
    //This is to fully intake a ball to the vertical conveyor

    mc_yButton.whileHeld(new RunIntakeMotor(m_intakeSubsystem, -1)); //reversed
    mc_yButton.whileHeld(new RunHorizontalConveyor(m_hConveyorSubsystem, 1)); //reversed
    //This is to reverse intake a ball out if not all the way in the conveyor

    mc_bButton.whileHeld(new RunIntakeMotor(m_intakeSubsystem, 1)); //reversed 

    mc_xButton.whenPressed(new DeployIntake(m_pneumaticSubsystem, m_intakeSubsystem));

    mc_lButton.whileHeld(new RunVerticalConveyor(m_vConveyorSubsystem, 0.8));
    mc_lButton.whileHeld(new SetShooterToSpeed(m_shooterSubsystem, 1234));
    //This is to spit out oppisite alliance ball 

    mc_rButton.whileHeld(new ShootTwoBalls(m_visionSubsystem, m_vConveyorSubsystem, m_intakeSubsystem, m_shooterSubsystem));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //Robot has to be lined up 40 inches away or closer from second ball pickup
     Command Backup = new TwoBallAuton(m_pneumaticSubsystem, m_intakeSubsystem, m_hConveyorSubsystem, m_drivetrainSubsystem, m_visionSubsystem, m_vConveyorSubsystem, m_shooterSubsystem);
     return Backup;
   }

    public Command getDisabledCommand(){
      Command disabled = new SetDisabledState(m_lighting);
      return disabled;
    } // Command to reset robot to initial state
    
    public Command getTeleopLightingCommand(){
      Command lightingCommand = new RunTeleopLighting(m_lighting, m_drivetrainSubsystem, m_visionSubsystem);
      return lightingCommand;
    }
}
 