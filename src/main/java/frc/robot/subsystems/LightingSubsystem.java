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
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase {
  private final CANdle light1 = new CANdle(Constants.CandleConstants.CANDLE_1_ID);
  private Animation m_animation = null;
  public boolean MexicanLEDOn = false; 

  /** Creates a new LightingSubsystem. */
  public LightingSubsystem() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.brightnessScalar = 0.6;
    cfg.vBatOutputMode = VBatOutputMode.Modulated;
    light1.configAllSettings(cfg);
    light1.configLEDType(LEDStripType.GRB);
    m_animation = new RainbowAnimation(0.7, 0.8, Constants.CandleConstants.CANDLE_1_LED_COUNT); 
    
  }

  public void setRainbowAnimation(){
    m_animation = new RainbowAnimation(0.7, 0.8, Constants.CandleConstants.CANDLE_1_LED_COUNT);
  } 

  public void setPurpleTwinkleAnimation(){
    m_animation = new TwinkleAnimation(225, 5, 225, 100, 0.8, Constants.CandleConstants.CANDLE_1_LED_COUNT, TwinklePercent.Percent76);
  }

  public void setRedTwinkleAnimation(){
    m_animation = new TwinkleAnimation(255, 0, 0, 0, 0.25, Constants.CandleConstants.CANDLE_1_LED_COUNT, TwinklePercent.Percent88); 
  }

  public void setBlueTwinkleAnimation(){
    m_animation = new TwinkleAnimation(0, 0, 225, 0, 0.25, Constants.CandleConstants.CANDLE_1_LED_COUNT, TwinklePercent.Percent88);
  }

  public void setDisabledLightShow(){
    setRainbowAnimation();
  }

  @Override
  public void periodic() {
    if (m_animation != null){
      light1.animate(m_animation);
    }

      
    // This method will be called once per scheduler run
  }

public void addOption(String string, Command command) {
}
}
