package frc.robot.subsystems;

import java.lang.Math;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.playingwithfusion.TimeOfFlight;

public class IntakeSubsystem extends SubsystemBase{

  // - - - - - - - - - - FIELDS AND CONSTRUCTORS - - - - - - - - - -

  private final PIDController m_pivotPID = new PIDController(
    IntakeConstants.kPivotP, 
    IntakeConstants.kPivotI, 
    IntakeConstants.kPivotD
  );

  private final PIDController m_ampPivotPID = new PIDController(
    IntakeConstants.kAmpPivotP, 
    IntakeConstants.kAmpPivotI, 
    IntakeConstants.kAmpPivotD
  );
  
  private final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(IntakeConstants.kPivotEncoderID);

  private CANSparkMax m_intakeMotor;
  private CANSparkMax m_pivotMotor;

  private TimeOfFlight m_timeOfFlight = new TimeOfFlight(IntakeConstants.kTimeOfFlightSensorID);

  private static IntakeSubsystem m_instance;

  private PivotState m_pivotTarget = PivotState.STOW;
  private PivotState m_pivotState = PivotState.NONE;
  private IntakeState m_intakeState = IntakeState.NONE;

  private double m_pivotSpeed = 0.0;
  private double m_intakeSpeed = 0.0;

  private boolean m_hasNote = false;

  /**
   * Constructor.
   */
  private IntakeSubsystem() {
    m_pivotPID.setTolerance(IntakeConstants.kPivotPIDTolerance);
    m_ampPivotPID.setTolerance(IntakeConstants.kAmpPivotPIDTolerance);

    // Intake Motor
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_intakeMotor.setInverted(IntakeConstants.kIntakeMotorInverted);
    
    // Pivot Motor
    m_pivotMotor = new CANSparkMax(IntakeConstants.kPivotMotorID, MotorType.kBrushless);

    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_pivotMotor.setSmartCurrentLimit(IntakeConstants.kPivotMotorSmartCurrentLimit);
    m_pivotMotor.setInverted(IntakeConstants.kPivotMotorInverted);
  }

  /**
   * Returns an instance of robot, this is an implementation of the singleton design pattern.
   * @return instance
   */
  public static IntakeSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeSubsystem();
    }
    return m_instance;
  }

  public enum PivotState {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    FEED_SPEAKER_SHOOTER,
    FEED_AMP_SHOOTER,
    AMP
  }

  // - - - - - - - - - - GENERIC FUNCTIONS - - - - - - - - - -
  
  @Override
  public void periodic(){

    m_hasNote = getTimeOfFlightRange() < IntakeConstants.kNoteIntakedSensorValue;

    // Pivot control
    double pivotAngle = pivotTargetToAngle(m_pivotTarget);
    m_pivotSpeed = m_pivotPID.calculate(getPivotAngleDegrees(), pivotAngle)/150;

    if (m_pivotTarget == PivotState.AMP){
      m_pivotSpeed = m_ampPivotPID.calculate(getPivotAngleDegrees(), pivotAngle)/100;
    }

    // Intake control
    m_intakeSpeed = (intakeStateToSpeed(m_intakeState));


    m_pivotMotor.set(m_pivotSpeed);
    m_intakeMotor.set(m_intakeSpeed);

    if (m_pivotPID.atSetpoint()){
      m_pivotState = m_pivotTarget;
    }

    SmartDashboard.putNumber("Intake Pivot Angle", getPivotAngleDegrees());
    SmartDashboard.putNumber("Intake Pivot Target", pivotAngle);
    SmartDashboard.putNumber("Intake Pivot Voltage", m_pivotSpeed);
    // SmartDashboard.putString("Intake Pivot State", m_intakeState.toString());
    
    SmartDashboard.putNumber("Intake Sensor Range", getTimeOfFlightRange());
    
    SmartDashboard.putData("Intake Pivot PID", m_pivotPID);
  }

  // - - - - - - - - - - PRIVATE FUNCTIONS - - - - - - - - - -

  /**
   * Returns the angle of the intake pivot
   * 
   * @return The angle of the intake pivot
   */
  private double getPivotAngleDegrees() {
    double value = m_pivotEncoder.getAbsolutePosition();

    value = reshiftAngle(Units.rotationsToDegrees(value%1>0?value%1:value%1+1));
    return value;
  }

  private double reshiftAngle(double angle){
    angle -= IntakeConstants.kPivotEncoderZero;
    while (angle < 0){
      angle += 360;
    }
    return angle;
  }

  /**
   * Converts PivotTarget enum to a target angle for the intake.
   * 
   * @param target The target state of the intake pivot
   * @return The target angle for the intake pivot
   */
  private double pivotTargetToAngle(PivotState state) {
    switch (state) {
      case GROUND:
        return IntakeConstants.kPivotAngleGround;
      case SOURCE:
        return IntakeConstants.kPivotAngleSource;
      case AMP:
        return IntakeConstants.kPivotAngleAmp;
      case STOW:
        return IntakeConstants.kPivotAngleStow;
      default:
        return 0;
    }
  }

  /**
   * Converts IntakeState enum to a motor speed for the intake.
   * 
   * @param state The state of the intake
   * @return Motor speed
   */
  private double intakeStateToSpeed(IntakeState state) {
    switch (state) {
      case INTAKE:
        return IntakeConstants.kIntakeSpeed;
      case EJECT:
        return IntakeConstants.kEjectSpeed;
      case FEED_SPEAKER_SHOOTER:
        return IntakeConstants.kFeedSpeakerShooterSpeed;
      case FEED_AMP_SHOOTER:
        return IntakeConstants.kFeedAmpShooterSpeed;
      case AMP:
        return IntakeConstants.kAmpSpeed;
      default:
        return 0.0;
    }
  }

  // - - - - - - - - - - PUBLIC FUNCTIONS - - - - - - - - - -

  public void setIntakeState(IntakeState state){
    m_intakeState = state;
  }

  public void setPivotTarget(PivotState target){
    m_pivotTarget = target;
  }

  public PivotState getPivotState(){
    return m_pivotState;
  }

  public IntakeState getIntakeState(){
    return m_intakeState;
  }

  public double getTimeOfFlightRange(){
    return m_timeOfFlight.getRange();
  }

  public boolean hasNote() {
    return m_hasNote;
  }

  // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/InstantCommand.html

  public static Command pivotCommand(PivotState state){
    return new RunCommand(
      () -> m_instance.setPivotTarget(state),
      m_instance
    ).withTimeout(0);
  }

  public static Command stateCommand(IntakeState state){
    return new RunCommand(
      () -> m_instance.setIntakeState(state),
      m_instance
    ).withTimeout(0);
  }
}
