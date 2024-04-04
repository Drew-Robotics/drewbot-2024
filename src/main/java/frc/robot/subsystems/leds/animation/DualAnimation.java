package frc.robot.subsystems.leds.animation;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.leds.Color;

public class DualAnimation extends AnimationBase {
  private Color m_color1, m_color2;

  private double time = 0.5;

  public DualAnimation(CANdle candle, Color color1, Color color2){
    super(candle);
    m_candle = candle;

    m_color1 = color1;
    m_color2 = color2;
  }

  public void update(){
    if (!animating){ return; }

    Color c1;
    Color c2;

    double currentTime = Timer.getFPGATimestamp();

    if ((currentTime % (time*2)) > time){
      c1 = m_color2;
      c2 = m_color1;
    } else{
      c1 = m_color1;
      c2 = m_color2;
    }

    double progress = (currentTime % time) / time;
    int startingLed = (int) (count * progress) + offset;
    
    setPixel(c1, offset, startingLed);
    setPixel(c2, startingLed, startingLed + count);
    setPixel(c1, startingLed + count, numLeds);
  }
}
