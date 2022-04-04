// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase {
  private final CANdle light1 = new CANdle(Constants.CandleConstants.CANDLE_1_ID);
  private final CANdle light2 = new CANdle(Constants.CandleConstants.CANDLE_2_ID);

  /** Creates a new LightingSubsystem. */
  public LightingSubsystem() {

  }

  public void setDisabledLightShow(){
    double brightness = 0.60; // from 0 to 1
    double speed = 0.6;
    double sparking = 0.6;
    double cooling = 0.6;
    light1.animate(
      new FireAnimation(
        brightness,
        speed,
        Constants.CandleConstants.CANDLE_1_LED_COUNT,
        sparking,
        cooling
      )
    );

  }

  public void setLightsOff(){
    // Not sure this is the best way to turn them off
    light1.configBrightnessScalar(0);
    light2.configBrightnessScalar(0);
  }

  public void setGreen(){
    light1.setLEDs(0, 255, 0);
    light2.setLEDs(0, 255, 0);
  }

  public void setOrange(){
    light1.setLEDs(255, 157, 0);
    light2.setLEDs(255, 157, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
