package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.lang.Math;
import java.util.function.DoubleSupplier;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.playingwithfusion.TimeOfFlight;

public class IntakeSubsystem extends SubsystemBase{

  // - - - - - - - - - - FIELDS AND CONSTRUCTORS - - - - - - - - - -

  // private ArmFeedforward m_pivotFF = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV, IntakeConstants.kA);

  private final DutyCycle m_pivotEncoder = new DutyCycle(new DigitalInput(IntakeConstants.kPivotEncoderID));

  private CANSparkMax m_intakeMotor;
  private CANSparkMax m_pivotMotor;

  private TimeOfFlight m_timeOfFlight = new TimeOfFlight(IntakeConstants.kTimeOfFlightSensorID);

  private static IntakeSubsystem m_instance;

  private PivotState m_pivotTarget = PivotState.STOW;
  private PivotState m_pivotState = PivotState.NONE;
  private IntakeState m_intakeState = IntakeState.NONE;

  private boolean m_hasNote = false;

  private double m_pivotVoltage = 0.0;
  private double m_intakeSpeed = 0.0;
  private double m_currentPivotPosRot = 0.0;
  private double m_prevPivotPosRot = 0.0;
  private double m_pivotVelRps = 0.0;
  private boolean m_characterizing = false;

  private double m_targetRot = 0;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(IntakeConstants.kMaxVelocityRps, IntakeConstants.kMaxAccelerationRpsps);
  private final ProfiledPIDController m_pivotPID =
      new ProfiledPIDController(IntakeConstants.kPivotP, IntakeConstants.kPivotI, IntakeConstants.kPivotD, m_constraints, TimedRobot.kDefaultPeriod);
  private final ProfiledPIDController m_pivotAmpPID =
      new ProfiledPIDController(IntakeConstants.kAmpPivotP, IntakeConstants.kAmpPivotI, IntakeConstants.kAmpPivotD, m_constraints, TimedRobot.kDefaultPeriod);

  private boolean atSetpoint = false;
  private boolean pivotMoving = false;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                m_pivotMotor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("pivot-motor")
                    .voltage(m_appliedVoltage.mut_replace(m_pivotMotor.getAppliedOutput() * m_pivotMotor.getBusVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(m_currentPivotPosRot, Rotations))
                    .angularVelocity(m_velocity.mut_replace(m_pivotVelRps, RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("intake")
              this));

  /**
   * Constructor.
   */
  private IntakeSubsystem() {
    m_pivotPID.setTolerance(IntakeConstants.kPivotPIDTolerance);
    m_pivotAmpPID.setTolerance(IntakeConstants.kAmpPivotPIDTolerance);

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
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    FEED_SPEAKER_SHOOTER,
    FEED_AMP_SHOOTER,
    HOLD,
    AMP
  }

  // - - - - - - - - - - GENERIC FUNCTIONS - - - - - - - - - -
  
  public void setTarget(double target){
    m_targetRot = target;
  }

  @Override
  public void periodic(){
    m_hasNote = getTimeOfFlightRange() < IntakeConstants.kNoteIntakedSensorValue;

    m_currentPivotPosRot = getPivotAngleDegrees() / 360.0;
    m_pivotVelRps = (m_currentPivotPosRot - m_prevPivotPosRot) / TimedRobot.kDefaultPeriod;
    

    // Pivot control
    double targetPivotRot = pivotTargetToDeg(m_pivotTarget) / 360;
    // targetPivotRot = m_targetRot;

    // double pivotFF = m_pivotFF.calculate(targetPivotRot * 2 * Math.PI, targetPivotRot - m_currentPivotPosRot);
    double pivotNonAmpVoltage = m_pivotPID.calculate(m_currentPivotPosRot, targetPivotRot);
    double pivotAmpVoltage = m_pivotAmpPID.calculate(m_currentPivotPosRot, targetPivotRot);

    m_pivotVoltage = (m_pivotTarget == PivotState.AMP) ? pivotAmpVoltage : pivotNonAmpVoltage;

    // Intake control
    m_intakeSpeed = (intakeStateToSpeed(m_intakeState));

    if(!m_characterizing) {
      m_pivotMotor.setVoltage((m_pivotVoltage));
      m_intakeMotor.set(m_intakeSpeed);
    }

    if (!atSetpoint && !pivotPidAtSetpoint()){
      pivotMoving = true;
    }

    if (pivotMoving && pivotPidAtSetpoint()){
      atSetpoint = true;
      pivotMoving = false;
    }

    if (atSetpoint){
      m_pivotState = m_pivotTarget;
    }

    SmartDashboard.putNumber("Intake Pivot Target", targetPivotRot);
    SmartDashboard.putString("Intake Pivot State", m_pivotState.toString());
    SmartDashboard.putBoolean("Intake Pivot At Setpoint", m_pivotPID.atSetpoint());
    SmartDashboard.putBoolean("Intake Amp Pivot At Setpoint", m_pivotAmpPID.atSetpoint());

    SmartDashboard.putData("Intake Pivot PID", m_pivotPID);
    
    SmartDashboard.putNumber("Intake Sensor Range", getTimeOfFlightRange());

    // m_pivotFF = new ArmFeedforward(0, 
    // (double) SmartDashboard.getEntry("Pivot FF: kG").getValue().getValue(),
    // (double) SmartDashboard.getEntry("Pivot FF: kV").getValue().getValue());

    m_prevPivotPosRot = m_currentPivotPosRot;
  }

  private boolean pivotPidAtSetpoint(){
    if (m_pivotTarget == PivotState.AMP){
      return m_pivotAmpPID.atSetpoint();
    }
    return m_pivotPID.atSetpoint();
  }

  // - - - - - - - - - - PRIVATE FUNCTIONS - - - - - - - - - -

  /**
   * Returns the angle of the intake pivot
   * 
   * @return The angle of the intake pivot
   */
  private double getPivotAngleDegrees() {
    return Units.rotationsToDegrees(getPivotRot());
  }

  private double getPivotRot(){
    // assumes that stow is the zero position and positive rotation MUST be from stow to ground.

    double encoderReading = m_pivotEncoder.getOutput();

    if (encoderReading < IntakeConstants.kPivotStowRotRaw - 0.1){ // 0.1 is tolerance for extending beyond the hardstop
      encoderReading++;
    }
    
    return encoderReading - IntakeConstants.kPivotStowRotRaw;
  }


  /**
   * Converts PivotTarget enum to a target angle for the intake.
   * 
   * @param target The target state of the intake pivot
   * @return The target angle for the intake pivot
   */
  private double pivotTargetToDeg(PivotState state) {
    switch (state) {
      case GROUND:
        return IntakeConstants.kPivotAngleGround;
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
    m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeDefaultAmps);
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
      case HOLD:
        m_intakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeHoldAmps);
        return IntakeConstants.kIntakeHoldSpeed;
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
    atSetpoint = false;
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

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction)
      .beforeStarting(() -> { 
        m_characterizing = true; 
      }, this)
      .andThen(() -> { 
        m_characterizing = false; 
      });
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction)
    .beforeStarting(() -> { 
      m_characterizing = true; 
    }, this)
    .andThen(() -> { 
      m_characterizing = false; 
    });
  }
}
