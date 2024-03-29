package frc.robot.subsystems.leds;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.Animation;

public class LEDState {

  private BooleanSupplier m_isActive;
  // private Animation m_activeAnimation;

  private int r, g, b;

  public LEDState(BooleanSupplier isActive, int r, int g, int b){
    m_isActive = isActive;
    this.r = r;
    this.g = g;
    this.b = b;
  }

  public int getR(){
    return r;
  }
  public int getG(){
    return g;
  }
  public int getB(){
    return b;
  }

  public boolean isActive(){
    return m_isActive.getAsBoolean();
  }

  // public Animation getAnimation(){
  //   return m_activeAnimation;
  // }
}
