// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ClimberSubsystem;

// public class Climber2 extends CommandBase {

//   private final ClimberSubsystem climber2;
//   double commandedOutputFraction;

//   /** Creates a new Climber2. */
//   public Climber2(ClimberSubsystem climberSubsystem, double commandedFraction) {
//     this.climber2 = climberSubsystem;
//     this.commandedOutputFraction = commandedFraction;
//     addRequirements(climber2);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     climber2.runClimberMotor(commandedOutputFraction);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     climber2.runClimberMotor(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
