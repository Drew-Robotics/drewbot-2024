package frc.robot.subsystems.leds;

import java.time.Period;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbersState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.PivotState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.leds.animation.DualAnimation;
import frc.robot.subsystems.leds.animation.SolidAnimation;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class LEDSubsystem extends SubsystemBase{
  private final CANdle m_candle = new CANdle(LEDConstants.CANdleID, "rio");

  private DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  private ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();
  private ClimberSubsystem m_climber = ClimberSubsystem.getInstance();

  private static LEDSubsystem m_instance;

  private ArrayList<LEDState> m_LEDStates = new ArrayList<LEDState>();

  BooleanSupplier trueSupplier = () -> true;
  BooleanSupplier shooterRevvingSupplier = () -> m_shooter.getShooterState() == ShooterState.SPEAKER;


  BooleanSupplier shootingSpeakerSupplier = () -> 
    (m_intake.getIntakeState() == IntakeState.FEED_SPEAKER_SHOOTER) &&
    (m_shooter.getShooterState() == ShooterState.SPEAKER);

  BooleanSupplier hasNoteSupplier = m_intake::hasNote;
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


  // change color values
  
  private Color hasNoteColor = new Color(255, 50, 0);

  private Color shootingSpeakerColor = new Color(255, 0, 200);
  private Color shootingAmpColor = new Color(0, 255, 0);

  private Color pivotDownColor = new Color(0, 255, 150);

  private Color climbersUpColor = new Color(255, 0, 255);
  private Color climbersUpSecondaryColor = new Color(255, 0, 255);

  private LEDState currentLEDState;


  private Color fieldColor = new Color(
    fieldColorSup(100, 0, 100), // r
    fieldColorSup(0, 0, 100), // g
    fieldColorSup(0, 100, 100) // b
  );

  public LEDSubsystem() {
    m_candle.setLEDs(0, 0, 0);
    
    m_LEDStates.add(new LEDState(shootingSpeakerSupplier, new SolidAnimation(m_candle, shootingSpeakerColor)));
    m_LEDStates.add(new LEDState(shooterRevvingSupplier, new DualAnimation(m_candle, shootingSpeakerColor, hasNoteColor)));

    m_LEDStates.add(new LEDState(hasNoteSupplier, new SolidAnimation(m_candle, hasNoteColor)));

    m_LEDStates.add(new LEDState(intakeGroundSupplier, new DualAnimation(m_candle, fieldColor, pivotDownColor)));

    m_LEDStates.add(new LEDState(climbersUpSupplier, new DualAnimation(m_candle, climbersUpColor, climbersUpSecondaryColor)));

    m_LEDStates.add(new LEDState(trueSupplier, new SolidAnimation(m_candle, fieldColor)));
  }

  private IntSupplier fieldColorSup(int r, int b, int none){
    return () -> {
      Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()){
        return (alliance.get() == DriverStation.Alliance.Red) ? r : b;
      }
      return none;
    };
  }

  @Override
  public void periodic(){
    checkLEDStates();
  }

  private void checkLEDStates(){
    if (currentLEDState != null){
      currentLEDState.update();
    }

    m_LEDStates.forEach(ledState -> { ledState.stop(); }); // stops all states to ensure that one is active

    for (LEDState LEDStateI : m_LEDStates){

      if (!LEDStateI.checkStatus()){continue;} // skips over non-active states and runs the ones that are.
      currentLEDState = LEDStateI;
      return;
    }
  }
}
