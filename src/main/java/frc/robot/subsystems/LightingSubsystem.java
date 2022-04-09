// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase {
  private final CANdle light1 = new CANdle(Constants.CandleConstants.CANDLE_1_ID);
  private Animation m_animation = null;
  //private final CANdle light2 = new CANdle(Constants.CandleConstants.CANDLE_2_ID);

  /** Creates a new LightingSubsystem. */
  public LightingSubsystem() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.brightnessScalar = 0.9;
    cfg.vBatOutputMode = VBatOutputMode.Modulated;
    light1.configAllSettings(cfg);
    light1.configLEDType(LEDStripType.GRB);
    
  }

  public void setRainbowAnimation(){
    m_animation = new RainbowAnimation(0.7, 0.6, Constants.CandleConstants.CANDLE_1_LED_COUNT);
    light1.animate(m_animation);
  } 

  public void setGreenSolidAnimation(){
    // light1.setLEDs(0, 255, 0, 10, 0, Constants.CandleConstants.CANDLE_1_LED_COUNT);
    m_animation = new ColorFlowAnimation(0, 255, 0, 10, 0.7, Constants.CandleConstants.CANDLE_1_LED_COUNT, Direction.Forward);
  }

  public void setOrangeSolidAnimation(){
    m_animation = new ColorFlowAnimation(255, 157, 0, 10, 0.7, Constants.CandleConstants.CANDLE_1_LED_COUNT, Direction.Forward);
  }

  public void setFireAnimation(){
    m_animation = new FireAnimation(0.9, 0.4, Constants.CandleConstants.CANDLE_1_LED_COUNT, 0.4, 0.2);
  }
  public void setTwinkleAnimation(){
    m_animation = new TwinkleAnimation(120, 69, 233, 10, 0.9, Constants.CandleConstants.CANDLE_1_LED_COUNT, TwinklePercent.Percent64);
  }
  public void setDisabledLightShow(){
    setRainbowAnimation();
  }

  public void setLightsOff(){
    // Not sure this is the best way to turn them off
    light1.configBrightnessScalar(0);
    //light2.configBrightnessScalar(0);
  }

  @Override
  public void periodic() {
    if (m_animation != null){
      light1.animate(m_animation);
    }
      
    // This method will be called once per scheduler run
  }
}
