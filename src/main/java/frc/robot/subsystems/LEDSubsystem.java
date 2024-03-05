package frc.robot.subsystems;

import java.time.Period;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ClimberSubsystem.ClimbersState;
import frc.robot.subsystems.IntakeSubsystem.PivotState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class LEDSubsystem extends SubsystemBase{
  private final int LEDS_PER_ANIMATION = 30;
  private final CANdle m_candle = new CANdle(LEDConstants.CANdleID, "rio");

  private Animation m_toAnimate = null;
  private LEDState m_LEDState = LEDState.NONE;
  

  private DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  private ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();
  private ClimberSubsystem m_climber = ClimberSubsystem.getInstance();

  private static LEDSubsystem m_instance;

  Trigger shooterAmpTrigger = new Trigger(() -> m_shooter.getShooterState() == ShooterState.AMP);
  Trigger shooterSpeakTrigger = new Trigger(() -> m_shooter.getShooterState() == ShooterState.SPEAKER);

  Trigger intakeHasNoteTrigger = new Trigger(m_intake::hasNote);
  Trigger intakeAmpTrigger = new Trigger(() -> m_intake.getPivotState() == PivotState.AMP);
  Trigger intakeGroundTrigger = new Trigger(() -> m_intake.getPivotState() == PivotState.GROUND);

  Trigger climbersUpTrigger = new Trigger(() -> m_climber.getClimberState() == ClimbersState.UP);
  
  /**
   * Returns an instance of robot, this is an implementation of the singleton design pattern.
   * @return instance
   */
  public static LEDSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new LEDSubsystem();
    }
    return m_instance;
  }


  public enum LEDState { 
    NONE,
    HAS_NOTE,
    NO_NOTE,
    NOTE_RECENT,
    INTAKE_AMP_SHOOT,
    SHOOTER_SHOOT,
    CLIMBING
  }

  public LEDSubsystem() {

  }

  public Command setLEDCommand() {
    return new RunCommand(
      () -> m_candle.setLEDs(255, 255, 255),
      m_instance
    ).withTimeout(1).andThen(
      () -> m_candle.setLEDs(0, 0, 0),
      m_instance
    );
  }

  @Override
  public void periodic(){}


}
