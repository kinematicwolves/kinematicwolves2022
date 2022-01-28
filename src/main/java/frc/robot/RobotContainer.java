// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BackupShootAuton;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.DriveRobotOpenLoop;
import frc.robot.commands.RunConveyorOpenLoop;
import frc.robot.commands.RunIntakeMotor;
import frc.robot.commands.RunShooterOpenLoop;
import frc.robot.commands.ShiftGear;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();

  // Controllers
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER);
  private final XboxController m_manipulatorController = new XboxController(Constants.MANIPULATOR_CONTROLLER);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    setDefaultCommands();
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
    JoystickButton dc_xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);
    JoystickButton mc_xButton = new JoystickButton(m_manipulatorController, XboxController.Button.kX.value);
    JoystickButton mc_aButton = new JoystickButton(m_manipulatorController, XboxController.Button.kA.value);
    JoystickButton dc_yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    JoystickButton mc_rButton = new JoystickButton(m_manipulatorController, XboxController.Button.kRightBumper.value);
    JoystickButton mc_lButton = new JoystickButton(m_manipulatorController, XboxController.Button.kLeftBumper.value);

    mc_xButton.whileHeld(new RunIntakeMotor(m_intakeSubsystem, Constants.DEFAULT_INTAKE_OUTPUT));
    mc_aButton.whileHeld(new RunIntakeMotor(m_intakeSubsystem, -1 * Constants.DEFAULT_INTAKE_OUTPUT));
    dc_xButton.whenPressed(new DeployIntake(m_pneumaticSubsystem, m_intakeSubsystem));
    dc_yButton.whenPressed(new ShiftGear(m_pneumaticSubsystem, m_drivetrainSubsystem));
    mc_rButton.whileHeld(new RunShooterOpenLoop(m_shooterSubsystem, 0.5));
    mc_lButton.whileHeld(new RunConveyorOpenLoop(m_conveyorSubsystem, 0.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
     // An ExampleCommand will run in autonomous
     Command Backup = new BackupShootAuton(m_drivetrainSubsystem);
     return Backup;
   }
}
