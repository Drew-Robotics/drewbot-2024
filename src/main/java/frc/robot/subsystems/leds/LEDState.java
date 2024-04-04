package frc.robot.subsystems.leds;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;

import frc.robot.subsystems.leds.animation.AnimationBase;
import frc.robot.subsystems.leds.animation.SolidAnimation;

public class LEDState {

  private BooleanSupplier m_isActive;

  private AnimationBase m_animation;

  // private Color m_color;
  // private Supplier<Color> m_colorSupplier;

  public LEDState(BooleanSupplier isActive, AnimationBase animation){
    m_isActive = isActive;
    m_animation = animation;
  }

  // public Color getColor(){
  //   if (m_color != null){
  //     return m_color;
  //   } else if (m_colorSupplier != null){
  //     return m_colorSupplier.get();
  //   }
  //   return null;
  // }

  public AnimationBase getAnimation(){
    return m_animation;
  }

  public boolean checkStatus(){
    boolean active = m_isActive.getAsBoolean();
    if (active){
      m_animation.start();
    } else{
      m_animation.stop();
    }

    return active;
  }
  
  public void stop(){
    m_animation.stop();
  }
}
