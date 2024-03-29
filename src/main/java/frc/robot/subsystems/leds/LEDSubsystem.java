package frc.robot.subsystems.leds;

import java.time.Period;
import java.util.function.BooleanSupplier;
import java.util.ArrayList;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbersState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.PivotState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class LEDSubsystem extends SubsystemBase{
  private final int LEDS_PER_ANIMATION = 30;
  private final CANdle m_candle = new CANdle(LEDConstants.CANdleID, "rio");

  // private Animation m_toAnimate = null;

  private DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  private ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();
  private ClimberSubsystem m_climber = ClimberSubsystem.getInstance();

  private static LEDSubsystem m_instance;

  private ArrayList<LEDState> m_LEDStates = new ArrayList<LEDState>();

  BooleanSupplier trueSupplier = () -> true;
  BooleanSupplier shooterAmpSupplier = () -> m_shooter.getShooterState() == ShooterState.AMP;
  BooleanSupplier shooterSpeakSupplier = () -> m_shooter.getShooterState() == ShooterState.SPEAKER;


  BooleanSupplier shooterSpeakerShootingSupplier = () -> 
    (m_intake.getIntakeState() == IntakeState.FEED_SPEAKER_SHOOTER) &&
    (m_shooter.getShooterState() == ShooterState.SPEAKER);

  BooleanSupplier intakeHasNoteSupplier = m_intake::hasNote;
  BooleanSupplier intakeAmpSupplier = () -> m_intake.getPivotState() == PivotState.AMP;
  BooleanSupplier intakeGroundSupplier = () -> m_intake.getPivotState() == PivotState.GROUND;

  BooleanSupplier climbersUpSupplier = () -> m_climber.getClimberState() == ClimbersState.UP;
  
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

  public LEDSubsystem() {

    m_LEDStates.add(new LEDState(shooterSpeakerShootingSupplier, 255, 0, 0));
    m_LEDStates.add(new LEDState(shooterSpeakSupplier, 100, 0, 100));

    m_LEDStates.add(new LEDState(intakeHasNoteSupplier, 0, 255, 255));

    m_LEDStates.add(new LEDState(climbersUpSupplier, 255, 0, 255));
    m_LEDStates.add(new LEDState(trueSupplier, 0, 255, 0));
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
  public void periodic(){
    checkLEDStates();
  }

  private void checkLEDStates(){
    for (LEDState LEDStateI : m_LEDStates){
      if (LEDStateI.isActive()){
        m_candle.setLEDs(
          LEDStateI.getR(),
          LEDStateI.getG(),
          LEDStateI.getB()
        );

        return;
      }
    }
  }


}
