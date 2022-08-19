package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/** 
 * Add your docs here. 
 */
 public class DriveStationJC extends Subsystem {
   // Put meothods for controlling this subsystem
   // Here. Call these from Commands. 


   edu.wpi.first.wpilibj.motorcontrol.Talon leftFrontTalon = null; 
   edu.wpi.first.wpilibj.motorcontrol.Talon leftBackTalon = null; 
   edu.wpi.first.wpilibj.motorcontrol.Talon rightfrontTalon = null; 
   edu.wpi.first.wpilibj.motorcontrol.Talon rightbackTalon = null; 


   public DriveStationJC() {
     // Talons 
     leftFrontTalon = new Talon(Constants static final.DRIVETRAIN_LEFT_FRONT_TALON);
     leftBackTalon = new Talon(Constants.DRIVETRAIN_LEFT_BACK_TALON);
     rightFrontTalon = new Talon(Constants.DRIVETRAIN_RIGHT_FRONT_TALON);
     rightBackTalon = new Talon(Constants.DRIVETRAIN_RIGHT_BACK_TALON);
     


   @Override 
   public void periodic() {
     // This method will be called once per scheduler run 
   }
 }