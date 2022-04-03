// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignWithTarget;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.DriveRobotOpenLoop;
import frc.robot.commands.RunClimber1OpenLoop;
import frc.robot.commands.RunHorizontalConveyor;
import frc.robot.commands.RunIntakeMotor;
import frc.robot.commands.RunVerticalConveyor;
import frc.robot.commands.SetShooterToSpeed;
import frc.robot.commands.ShiftGear;
import frc.robot.commands.ShootTwoBalls;
import frc.robot.commands.ShootWithLTrigger;
import frc.robot.commands.ToggleSpeedLimit;
import frc.robot.commands.TwoBallAuton;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
    m_shooterSubsystem.setDefaultCommand(new ShootWithLTrigger( m_shooterSubsystem, m_manipulatorController, m_visionSubsystem)); //Left Trigger runs conveyor and shooter
    //m_shooterSubsystem.setDefaultCommand(new ShootWithRTrigger(m_shooterSubsystem, m_manipulatorController));
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
    JoystickButton dc_leftStickButton = new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value);
    JoystickButton dc_rButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    JoystickButton dc_xButton = new JoystickButton(m_driverController, XboxController.Button.kX.value);

    JoystickButton dc_yButton = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    JoystickButton mc_aButton = new JoystickButton(m_manipulatorController, XboxController.Button.kA.value);
    JoystickButton mc_rButton = new JoystickButton(m_manipulatorController, XboxController.Button.kRightBumper.value);
    JoystickButton mc_lButton = new JoystickButton(m_manipulatorController, XboxController.Button.kLeftBumper.value);
    JoystickButton mc_xButton = new JoystickButton(m_manipulatorController, XboxController.Button.kX.value);
    JoystickButton mc_yButton = new JoystickButton(m_manipulatorController, XboxController.Button.kY.value);
    JoystickButton mc_bButton = new JoystickButton(m_manipulatorController, XboxController.Button.kB.value);
    JoystickButton mc_lefJoystickButton = new JoystickButton(m_manipulatorController, XboxController.Button.kLeftStick.value);



    //Driver Controller
    dc_aButton.whenPressed(new ShiftGear(m_pneumaticSubsystem, m_drivetrainSubsystem));
    dc_leftStickButton.whenPressed(new ToggleSpeedLimit(m_drivetrainSubsystem));
    dc_yButton.whileHeld(new RunClimber1OpenLoop(m_climberSubsystem, -0.2));
    dc_bButton.whileHeld(new RunClimber1OpenLoop(m_climberSubsystem, -0.55));
    dc_rButton.whileHeld(new AlignWithTarget(m_visionSubsystem, m_drivetrainSubsystem, 0.31));
    dc_xButton.whileHeld(new ShootTwoBalls(m_visionSubsystem, m_vConveyorSubsystem, m_intakeSubsystem, m_shooterSubsystem));
    //Munipulator Controller 
    //-RunIntakeMotor = Horizontal Conveyor
    //-RunHorizontalConveyor = Intake Motor
    mc_aButton.whileHeld(new RunIntakeMotor(m_intakeSubsystem, -1));
    mc_aButton.whileHeld(new RunHorizontalConveyor(m_hConveyorSubsystem, -1));
    mc_aButton.whileHeld(new RunVerticalConveyor(m_vConveyorSubsystem, 0.1));
    mc_yButton.whileHeld(new RunIntakeMotor(m_intakeSubsystem, -1)); //reversed
    mc_yButton.whileHeld(new RunHorizontalConveyor(m_hConveyorSubsystem, 1)); //reversed
    mc_bButton.whileHeld(new RunIntakeMotor(m_intakeSubsystem, 1)); //reversed 
    mc_xButton.whenPressed(new DeployIntake(m_pneumaticSubsystem, m_intakeSubsystem));
    mc_rButton.whileHeld(new RunVerticalConveyor(m_vConveyorSubsystem, 0.8));
    mc_lButton.whileHeld(new RunIntakeMotor(m_intakeSubsystem, -1));
    mc_lefJoystickButton.whileHeld(new SetShooterToSpeed(m_shooterSubsystem, 1000));
   // mc_lButton.whileHeld(new AlignWithTarget(m_visionSubsystem, m_drivetrainSubsystem, 0.31));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
     Command Backup = new TwoBallAuton(m_pneumaticSubsystem, m_intakeSubsystem, m_hConveyorSubsystem, m_drivetrainSubsystem, m_visionSubsystem, m_vConveyorSubsystem, m_shooterSubsystem);
     return Backup;
   }

  public Command getRamsetCommand() {
      // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.PathPlanningConstants.ksVolts,
            Constants.PathPlanningConstants.kvVoltSecondsPerMeter,
            Constants.PathPlanningConstants.kaVoltSecondsSquaredPerMeter),
        m_drivetrainSubsystem.kinematics,
        10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.PathPlanningConstants.kMaxSpeedMetersPerSecond,
                Constants.PathPlanningConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_drivetrainSubsystem.kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_drivetrainSubsystem::getPose,
            new RamseteController(Constants.PathPlanningConstants.kRamseteB, Constants.PathPlanningConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.PathPlanningConstants.ksVolts,
                Constants.PathPlanningConstants.kvVoltSecondsPerMeter,
                Constants.PathPlanningConstants.kaVoltSecondsSquaredPerMeter),
            m_drivetrainSubsystem.kinematics,
            m_drivetrainSubsystem::getDifferentialDriveWheelSpeeds,
            new PIDController(Constants.PathPlanningConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.PathPlanningConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrainSubsystem::tankDriveVolts, 
            m_drivetrainSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0));
    }



    /*public Command getDisabledCommand() // Command to reset robot to initial state
    }*/
}
 