package frc.robot.subsystems;

import java.lang.Math;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.playingwithfusion.TimeOfFlight;

public class IntakeSubsystem extends SubsystemBase{

  // - - - - - - - - - - FIELDS AND CONSTRUCTORS - - - - - - - - - -

  private final PIDController m_pivotPID = new PIDController(IntakeConstants.kPivotP, IntakeConstants.kPivotI, IntakeConstants.kPivotD);
  private final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(IntakeConstants.kPivotEncoderId);

  private CANSparkMax m_intakeMotor;
  private CANSparkMax m_pivotMotor;

  private TimeOfFlight m_timeOfFlight = new TimeOfFlight(IntakeConstants.kTimeOfFlightSensorID);

  private static IntakeSubsystem m_instance;

  public static PeriodicIO m_periodicIO;
    public PeriodicIO getpPeriodicIO(){return m_periodicIO;}

  /**
   * Constructor.
   */
  private IntakeSubsystem() {
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

    m_periodicIO = new PeriodicIO(); 
  }

  /**
   * Periodic input output javadoc filler
   */
  private static class PeriodicIO {
    private PivotTarget pivotTarget = PivotTarget.STOW;
    private IntakeState intakeState = IntakeState.NONE;

    private double intakePivotVoltage = 0.0;
    private double intakeSpeed = 0.0;

    // Pivot target
    public PivotTarget getPivotTarget(){
      return pivotTarget;
    }
    public void setPivotTarget(PivotTarget pivotTarget){
      this.pivotTarget = pivotTarget;
    }

    // Intake state
    public IntakeState getIntakeState(){
      return intakeState;
    }
    public void setIntakeState(IntakeState intakeState){
      this.intakeState = intakeState;
    }

    // Pivot Voltage
    public double getIntakePivotVoltage(){
      return intakePivotVoltage;
    }
    public void setIntakePivotVoltage(double intakePivotVoltage){
      this.intakePivotVoltage = intakePivotVoltage;
    }

    // Intake speed
    public double getIntakeSpeed(){
      return intakeSpeed;
    }
    public void setIntakeSpeed(double intakeSpeed){
      this.intakeSpeed = intakeSpeed;
    }
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

  public enum PivotTarget {
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
    FEED_SHOOTER,
  }

  // - - - - - - - - - - GENERIC FUNCTIONS - - - - - - - - - -
  
  @Override
  public void periodic(){
    // Pivot control
    double pivotAngle = pivotTargetToAngle(m_periodicIO.getPivotTarget());
    m_periodicIO.setIntakePivotVoltage(m_pivotPID.calculate(getPivotAngleDegrees(), pivotAngle));

    // Intake control
    m_periodicIO.setIntakeSpeed(intakeStateToSpeed(m_periodicIO.getIntakeState()));


    // 
    m_pivotMotor.setVoltage(m_periodicIO.getIntakePivotVoltage());
    m_intakeMotor.set(m_periodicIO.getIntakeSpeed());

    SmartDashboard.putNumber("getPivotAngleDegrees", getPivotAngleDegrees());
    SmartDashboard.putNumber("getIntakePivotVoltage", m_periodicIO.getIntakePivotVoltage());
    SmartDashboard.putString("intakePivotTarget", m_periodicIO.getPivotTarget().toString());
    
    SmartDashboard.putNumber("timeOfFlightSensorRange", getTimeOfFlightRange());
    SmartDashboard.putString("timeOfFlightSensorStatus", m_timeOfFlight.getStatus().toString());

  }

  // - - - - - - - - - - PRIVATE FUNCTIONS - - - - - - - - - -

  /**
   * Returns the angle of the intake pivot
   * 
   * @return The angle of the intake pivot
   */
  private double getPivotAngleDegrees() {
    double value = m_pivotEncoder.getAbsolutePosition() - IntakeConstants.kPivotEncoderZero;

    value = Units.rotationsToDegrees(value%1>0?value%1:value%1+1);

    while (value < 0){
      value += 360;
    }
    return value;
  }

  /**
   * Converts PivotTarget enum to a target angle for the intake.
   * 
   * @param target The target state of the intake pivot
   * @return The target angle for the intake pivot
   */
  private double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return IntakeConstants.kPivotAngleGround;
      case SOURCE:
        return IntakeConstants.kPivotAngleSource;
      case AMP:
        return IntakeConstants.kPivotAngleAmp;
      case STOW:
        return IntakeConstants.kPivotAngleStow;
      default:
        return 180;
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
      case FEED_SHOOTER:
        return IntakeConstants.kFeedShooterSpeed;
      default:
        return 0.0;
    }
  }

  // - - - - - - - - - - PUBLIC FUNCTIONS - - - - - - - - - -

  public void setIntakeState(IntakeState state){
    m_periodicIO.setIntakeState(state);
  }

  public void setPivotTarget(PivotTarget target){
    m_periodicIO.setPivotTarget(target);
  }

  public double getTimeOfFlightRange(){
    return m_timeOfFlight.getRange();
  }
}
