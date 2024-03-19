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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

  private final ArmFeedforward m_pivotFF = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV, IntakeConstants.kA);

  private final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(IntakeConstants.kPivotEncoderID);

  private CANSparkMax m_intakeMotor;
  private CANSparkMax m_pivotMotor;

  private TimeOfFlight m_timeOfFlight = new TimeOfFlight(IntakeConstants.kTimeOfFlightSensorID);

  private static IntakeSubsystem m_instance;

  private PivotState m_pivotTarget = PivotState.STOW;
  private PivotState m_pivotState = PivotState.NONE;
  private IntakeState m_intakeState = IntakeState.NONE;

  private boolean m_hasNote = false;

  private double m_pivotFeedback = 0.0;
  private double m_intakeSpeed = 0.0;
  private double m_currentPivotPosRot = 0.0;
  private double m_prevPivotPosRot = 0.0;
  private double m_pivotVelRps = 0.0;
  private boolean m_characterizing = false;

  // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(IntakeConstants.kMaxVelocityRps, IntakeConstants.kMaxAccelerationRpsps);
  private final ProfiledPIDController m_pivotPID =
      new ProfiledPIDController(IntakeConstants.kPivotP, IntakeConstants.kPivotI, IntakeConstants.kPivotD, m_constraints, TimedRobot.kDefaultPeriod);

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
  
  @Override
  public void periodic(){
    m_hasNote = getTimeOfFlightRange() < IntakeConstants.kNoteIntakedSensorValue;

    m_currentPivotPosRot = getPivotAngleDegrees() / 360.0;
    m_pivotVelRps = (m_currentPivotPosRot - m_prevPivotPosRot) / TimedRobot.kDefaultPeriod;
    

    // Pivot control
    double targetPivotRot = pivotTargetToDeg(m_pivotTarget) / 360;

    double pivotFF = m_pivotFF.calculate(targetPivotRot, 0);
    m_pivotFeedback = m_pivotPID.calculate(m_currentPivotPosRot, targetPivotRot);

    // Intake control
    m_intakeSpeed = (intakeStateToSpeed(m_intakeState));

    if(!m_characterizing) {
      m_pivotMotor.setVoltage((pivotFF + m_pivotFeedback));
      m_intakeMotor.set(m_intakeSpeed);
    }

    if (m_pivotPID.atSetpoint()){
      m_pivotState = m_pivotTarget;
    }

    SmartDashboard.putNumber("Intake Pivot Rot", getPivotAngleDegrees() / 360);
    SmartDashboard.putNumber("Intake Pivot Setpoint", targetPivotRot);
    SmartDashboard.putNumber("Intake Pivot Total Applied Voltage", pivotFF + m_pivotFeedback);
    SmartDashboard.putNumber("Intake Pivot FF Applied Voltage", pivotFF);
    SmartDashboard.putNumber("Intake Pivot PID Applied Voltage", m_pivotFeedback);
    // SmartDashboard.putString("Intake Pivot State", m_intakeState.toString());
    
    
    SmartDashboard.putNumber("Intake Sensor Range", getTimeOfFlightRange());
    
    SmartDashboard.putData("Intake Pivot PID", m_pivotPID);

    m_prevPivotPosRot = m_currentPivotPosRot;
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
