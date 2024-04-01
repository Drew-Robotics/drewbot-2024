package frc.robot.subsystems.leds;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix.led.Animation;

public class LEDState {

  private BooleanSupplier m_isActive;

  public enum LEDStateType {
    COLOR,
    ANIMATION
  }
  private LEDStateType m_ledStateType;

  private Integer m_r, m_g, m_b;
  private Animation m_Animation;
  private IntSupplier m_rSupplier, m_gSupplier, m_bSupplier;

  public LEDState(BooleanSupplier isActive, Integer r, Integer g, Integer b){
    m_isActive = isActive;

    m_ledStateType = LEDStateType.COLOR;
    m_r = r;
    m_g = g;
    m_b = b;
  }

  public LEDState(BooleanSupplier isActive, IntSupplier rSupplier, IntSupplier gSupplier, IntSupplier bSupplier){
    m_isActive = isActive;

    m_ledStateType = LEDStateType.COLOR;
    m_rSupplier = rSupplier;
    m_gSupplier = gSupplier;
    m_bSupplier = bSupplier;
  }

  public LEDState(BooleanSupplier isActive, Animation animation){
    m_isActive = isActive;

    m_ledStateType = LEDStateType.COLOR;
    m_Animation = animation;
  }

  public Integer getR(){
    if (m_r != null){
      return m_r;
    } else if (m_rSupplier != null){
      return m_rSupplier.getAsInt();
    }
    return null;
  }

  public Integer getG(){
    if (m_g != null){
      return m_g;
    } else if (m_gSupplier != null){
      return m_gSupplier.getAsInt();
    }
    return null;
  }

  public Integer getB(){
    if (m_b != null){
      return m_b;
    } else if (m_bSupplier != null){
      return m_bSupplier.getAsInt();
    }
    return null;
  }
  
  public boolean isActive(){
    return m_isActive.getAsBoolean();
  }

  public Animation getAnimation(){
    return m_Animation;
  }
}
