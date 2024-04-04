package frc.robot.subsystems.leds.animation;

import com.ctre.phoenix.led.CANdle;

import frc.robot.subsystems.leds.Color;

public class AnimationBase{
  protected int numLeds = 25;
  protected int count = 25;
  protected int offset = 12;
  protected double time = 5;

  protected CANdle m_candle;
  protected boolean animating = false;

  protected AnimationBase(CANdle candle){
    m_candle = candle;
  }

  public void update() throws Exception{
    throw new Exception("override this");
  }

  protected void setPixel(Color color, int from, int to){
    m_candle.setLEDs(
      color.getR(),
      color.getG(),
      color.getB(), 0, from, to - from);
  }

  public void start(){
    animating = true;
  }

  public void stop(){
    animating = false;
  }
}
