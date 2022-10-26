// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase {
  private final CANdle light1 = new CANdle(Constants.CandleConstants.CANDLE_1_ID);
  private Animation m_animation = null;

  /** Creates a new LightingSubsystem. */
  public LightingSubsystem() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.brightnessScalar = 1;
    cfg.vBatOutputMode = VBatOutputMode.Modulated;
    light1.configAllSettings(cfg);
    light1.configLEDType(LEDStripType.GRB);
  }

  //Animations
  public void setRainbowAnimation(){
    m_animation = new RainbowAnimation(0.8, 0.88, Constants.CandleConstants.CANDLE_1_LED_COUNT);
  }
  public void setPurpleTwinkleAnimation(){
    m_animation = new TwinkleAnimation(255, 0, 255, 69, 0.9, Constants.CandleConstants.CANDLE_1_LED_COUNT, TwinklePercent.Percent100); 
  }
  public void setRedTwinkleAnimation(){
    m_animation = new TwinkleAnimation(225, 0, 1, 0, 0.9, Constants.CandleConstants.CANDLE_1_LED_COUNT,TwinklePercent.Percent100);
  }
  public void setBlueTwinkleAnimation(){
    m_animation = new TwinkleAnimation(0, 0, 225, 30, 0.9, Constants.CandleConstants.CANDLE_1_LED_COUNT,TwinklePercent.Percent100);
  }
  public void setGreenTwinkleAnimation(){
    m_animation = new TwinkleAnimation(0, 225, 0, 0, 0.9, Constants.CandleConstants.CANDLE_1_LED_COUNT,TwinklePercent.Percent100); 
  }
  public void setBlackAnimation(){
    m_animation = new TwinkleAnimation(0, 0, 0, 0 , 0 , Constants.CandleConstants.CANDLE_1_LED_COUNT,TwinklePercent.Percent100); 
  }

  public void setDisabledLightShow(){
    m_animation = new RainbowAnimation(0.35, 0.3, Constants.CandleConstants.CANDLE_1_LED_COUNT);
  }

  @Override
  public void periodic() {
    if (m_animation != null){
      light1.animate(m_animation);
    }
  }
}
