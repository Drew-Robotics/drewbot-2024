package frc.robot.subsystems.leds;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.Animation;

public class LEDState {

  private BooleanSupplier m_isActive;
  private Animation m_activeAnimation;

  public LEDState(BooleanSupplier isActive, Animation activeAnimation){
    m_isActive = isActive;
    m_activeAnimation = activeAnimation;
  }

  public boolean isActive(){
    return m_isActive.getAsBoolean();
  }

  public Animation getAnimation(){
    return m_activeAnimation;
  }
}
