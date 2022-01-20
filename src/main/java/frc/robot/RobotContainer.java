// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Backup_Shoot_Auton;
import frc.robot.commands.Backwords_Auton;
import frc.robot.commands.DriveRobotOpenLoop;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;

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
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
     // An ExampleCommand will run in autonomous
     Command Backup = new Backup_Shoot_Auton(m_drivetrainSubsystem, null, null, null);
     return Backup;
   }
}
