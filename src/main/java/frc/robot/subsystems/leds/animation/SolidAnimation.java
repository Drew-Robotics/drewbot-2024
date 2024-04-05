package frc.robot.subsystems.leds.animation;

import java.util.function.Supplier;
import com.ctre.phoenix.led.CANdle;
import frc.robot.subsystems.leds.Color;

public class SolidAnimation extends AnimationBase{
  private Color m_color;
  private Supplier<Color> m_colorSupplier;

  public SolidAnimation(CANdle candle, Color color){
    super(candle);
    m_candle = candle;

    m_color = color;
  }

  public void update(){
    if (!animating){
      return;
    }
    setPixel(m_color, offset, count + offset);
  }
}
