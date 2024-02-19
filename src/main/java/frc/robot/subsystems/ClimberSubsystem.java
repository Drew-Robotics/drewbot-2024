package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  // Checking Instance
  private static ClimberSubsystem m_instance;
  public static ClimberSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ClimberSubsystem();
    }
    return m_instance;
  }

  private PeriodicIO m_periodicIO;

  // Defining Motors
  private CANSparkMax m_leftClimberMotor;
  private CANSparkMax m_rightClimberMotor;

  // Defining Motor PIDs
  private SparkPIDController m_leftClimberMotorPID;
  private SparkPIDController m_rightClimberMotorPID;

  // Defining Motor Encoders
  private RelativeController m_leftClimberEncoder;
  private RelativeController m_rightClimberEncoder;

  private ClimberSubsystem() {
    
    m_periodicIO = new PeriodicIO();
    
    // Setting Motor Type/ID
    m_leftClimberMotor = new CANSparkMax(ClimberConstants.kLeftClimberMotorID, MotorType.kBrushless);
    m_rightClimberMotor = new CANSparkMax(ClimberConstants.kRightClimberMotorId, MotorType.kBrushless);
    
    // Left Climber PID
    m_leftClimberMotorPID = m_leftClimberMotor.getPIDController();
    m_leftClimberMotorPID.setP(ClimberConstants.kClimberP);
    m_leftClimberMotorPID.setI(ClimberConstants.kClimberI);
    m_leftClimberMotorPID.setD(ClimberConstants.kClimberD);
    m_leftClimberMotorPID.setOutputRange(ClimberConstants.kClimberMinOutput, ClimberConstants.kClimberMaxOutput);

    // Right Climber PID
    m_rightClimberMotorPID = m_rightClimberMotor.getPIDController();
    m_rightClimberMotorPID.setP(ClimberConstants.kClimberP);
    m_rightClimberMotorPID.setI(ClimberConstants.kClimberI);
    m_rightClimberMotorPID.setD(ClimberConstants.kClimberD);  
    m_rightClimberMotorPID.setOutputRange(ClimberConstants.kClimberMinOutput, ClimberConstants.kClimberMaxOutput);

    // Left Climber Encoder
    m_leftClimberEncoder = m_leftClimberMotor.getEncoder();
    m_leftClimberEncoder.setPositionConversionFactor(ClimberConstants.kClimberGearRatio);
    m_leftClimberEncoder.setVelocityConversionFactor(ClimberConstants.kClimberGearRatio);

    // Right Climber Encoder
    m_rightClimberEncoder = m_rightClimberMotor.getEncoder();
    m_rightClimberEncoder.setPositionConversionFactor(ClimberConstants.kClimberGearRatio);
    m_rightClimberEncoder.setPositionConversionFactor(ClimberConstants.kClimberGearRatio);

    // Idle Mode 
    m_leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Inverting Motors
    m_leftClimberMotor.setInverted(false);
    m_rightClimberMotor.setInverted(true);

  }

  // RPM of Spool (whatever that means )
  private static class PeriodicIO {
    double climber_right_speed = 0.0;
    double climber_left_speed = 0.0;
  }
  
  /* ~ Subsystem Functions ~ */ 
  
  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    m_leftClimberMotorPID.setReference(m_periodicIO.climber_left_speed, ControlType.kVelocity);
    m_rightClimberMotorPID.setReference(m_periodicIO.climber_right_speed, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    stopClimber();
  }
  
  @Override
  public void outputTelemetry() {
    putNumber("Left speed setpoint:", m_periodicIO.climber_left_speed);
    putNumber("Left speed:", m_leftClimberEncoder.getVelocity());
    putNumber("Right speed setpoint:", m_periodicIO.climber_right_speed);
    putNumber("Right speed:", m_rightClimberEncoder.getVelocity());
  }

  @Override
  public void reset() {
  }
  
  /* ~ Custom Functions (by CA) ~ */ 

  public void setBrakeMode() {
    m_leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setCoastMode() {
    m_leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }
  
  // Climb :)
  public void climb() {
    m_periodicIO.climber_left_speed = ClimberConstants.kClimberClimbSpeed;
    m_periodicIO.climber_right_speed = ClimberConstants.kClimberClimbSpeed;
  }
  
  // Release
  public void release() {
    m_periodicIO.climber_left_speed = ClimberConstants.kClimberReleaseSpeed;
    m_periodicIO.climber_right_speed = ClimberConstants.kClimberReleaseSpeed;
  }
  
  public void tiltLeft() {
    m_periodicIO.climber_left_speed = ClimberConstants.kClimberReleaseSpeed;
    m_periodicIO.climber_right_speed = 0.0;
  }

  public void tiltRight() {
    m_periodicIO.climber_left_speed = 0.0;
    m_periodicIO.climber_right_speed = ClimberConstants.kClimberReleaseSpeed;
  }

  public void stopClimber() {
    m_periodicIO.climber_left_speed = 0.0;
    m_periodicIO.climber_right_speed = 0.0;
  }

}

/* ;-; ClimberConstants Because git ;-; */ 

/*

  public static final class ClimberConstants {

    // PIDs 
    public static final double kClimberP = 0;
    public static final double kClimberI = 0;
    public static final double kClimberD = 0;

    // Motor IDs
    public static final int kLeftClimberMotorID = 0;
    public static final int kRightClimberMotorID = 0;

    // RPM
    public static final double kClimberClimbSpeed = 0.0;
    public static final double kClimberReleaseSpeed = 0.0;
    
    // Output Min/Max 
    public static final double kClimberMinOutput = 0;
    public static final double kClimberMaxOutput = 0;

    // Gear Ratio
    public static final double kClimberGearRatio = 0 / 0;
  }
                                                               */
