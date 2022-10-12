// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * This Command sequence is supposed to line up and shoot 2 balls with 1 button. 
 * (Without looking at the code) The sequence is as follows: 
 * 
 * 
 * AlignWithTarget Command:
 *  - Shooter is set to 4000 RPM (This is to reduce the amount of time is 
 *    takes the shooter to get to the necessary speed for the shot)
 *  - Compressor Turn off (To conserve battery health)
 *  - Deploy Intake (This is to make sure the second ball doesn't fall out during seperation between the 2)
 *  - Aligns with target
 *  - Sets shooter speed to necessary speed for shot
 * ShootTwoBalls Command:
 *  - Reverses Horizontal Conveyor for 1 second to seperate the balls
 *  - Runs vertical conveyor for 0.6 seconds to lift the ball into the shooter flywheel (shoots first ball)
 *  - Stops vertical to stop it from pushing the second ball into the deadzone
 *  - Launch's second ball from intake to vertical [0.6 seconds]
 *  - Vertical conveyor lifts ball to shooter flywheel (shoots second ball) while horizontal continues to 
 *    run incase the ball moves to deadzone
 *  - All motors get set to 0 output
 *  - Compressor enables
 *  
 */

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.HConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VConveyorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallShot extends ParallelRaceGroup {
  private ShooterSubsystem shooter;
  private VConveyorSubsystem vconveyor;
  private HConveyorSubsystem horizontal;
  private IntakeSubsystem intake;
  private PneumaticSubsystem pneumatics;

  /** Creates a new TwoBallShot. */
  public TwoBallShot(DifferentialDrivetrain drivetrain, PneumaticSubsystem pneumatics, 
  IntakeSubsystem intake, VisionSubsystem vision, LightingSubsystem lighting, HConveyorSubsystem horizontal, 
  VConveyorSubsystem vconveyor, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignWithTarget(vision, drivetrain, shooter, pneumatics, intake, 0.36), // DO NOT CHANGE SPEED
      new ShootTwoBalls(vision, vconveyor, horizontal, shooter, intake, pneumatics)
    );
  }
/*Try this if the motors dont stop running*/
  // @Override
  // public void end(boolean interrupted) {
  //     new ShootingSequenceEmergencyStop(shooter, vconveyor, horizontal, intake, pneumatics);
  // }
}
