package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  // - - - - - - - - - - FIELDS AND CONSTRUCTORS - - - - - - - - - -

  private static ClimberSubsystem m_instance;
  private PeriodicIO m_periodicIO;

  // Defining Motors
  private CANSparkMax m_leftClimberMotor;
  private CANSparkMax m_rightClimberMotor;

  // Defining Motor PIDs
  private SparkPIDController m_leftClimberMotorPID;
  private SparkPIDController m_rightClimberMotorPID;

  // Defining Motor Encoders
  private RelativeEncoder m_leftClimberEncoder;
  private RelativeEncoder m_rightClimberEncoder;

  private ClimberSubsystem() {
    
    m_periodicIO = new PeriodicIO();
    
    // Setting motor type/ID
    m_leftClimberMotor = new CANSparkMax(ClimberConstants.kLeftClimberMotorID, MotorType.kBrushless);
    m_rightClimberMotor = new CANSparkMax(ClimberConstants.kRightClimberMotorID, MotorType.kBrushless);

    // Set inverted
    m_leftClimberMotor.setInverted(ClimberConstants.kLeftClimberMotorInverted);
    m_rightClimberMotor.setInverted(ClimberConstants.kRightClimberMotorInverted);
    
    // Left climber PID
    m_leftClimberMotorPID = m_leftClimberMotor.getPIDController();
    m_leftClimberMotorPID.setP(ClimberConstants.kClimberP);
    m_leftClimberMotorPID.setI(ClimberConstants.kClimberI);
    m_leftClimberMotorPID.setD(ClimberConstants.kClimberD);
    m_leftClimberMotorPID.setOutputRange(ClimberConstants.kClimberMinOutput, ClimberConstants.kClimberMaxOutput);

    // Right climber PID
    m_rightClimberMotorPID = m_rightClimberMotor.getPIDController();
    m_rightClimberMotorPID.setP(ClimberConstants.kClimberP);
    m_rightClimberMotorPID.setI(ClimberConstants.kClimberI);
    m_rightClimberMotorPID.setD(ClimberConstants.kClimberD);  
    m_rightClimberMotorPID.setOutputRange(ClimberConstants.kClimberMinOutput, ClimberConstants.kClimberMaxOutput);

    // Left climber encoder
    m_leftClimberEncoder = m_leftClimberMotor.getEncoder();
    m_leftClimberEncoder.setPositionConversionFactor(ClimberConstants.kClimberGearRatio);
    m_leftClimberEncoder.setVelocityConversionFactor(ClimberConstants.kClimberGearRatio);

    // Right climber encoder
    m_rightClimberEncoder = m_rightClimberMotor.getEncoder();
    m_rightClimberEncoder.setPositionConversionFactor(ClimberConstants.kClimberGearRatio);
    m_rightClimberEncoder.setPositionConversionFactor(ClimberConstants.kClimberGearRatio);

    // Idle mode 
    m_leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Inverting motors
    m_leftClimberMotor.setInverted(false);
    m_rightClimberMotor.setInverted(true);

  }


  private static class PeriodicIO {
    private double climberRightSpeed = 0.0;
    private double climberLeftSpeed = 0.0;

    public double getClimberRightSpeed(){
      return climberRightSpeed;
    }
    public void setClimberRightSpeed(double speed){
      climberRightSpeed = speed;
    }

    public double getClimberLeftSpeed(){
      return climberLeftSpeed;
    }
    public void setClimberLeftSpeed(double speed){
      climberLeftSpeed = speed;
    }
  }
  
  /**
   * Returns an instance of robot, this is an implementation of the singleton design pattern.
   * @return instance
   */
  public static ClimberSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ClimberSubsystem();
    }
    return m_instance;
  }

  // - - - - - - - - - - GENERIC FUNCTIONS - - - - - - - - - -
  
  @Override
  public void periodic() {
    // m_leftClimberMotorPID.setReference(m_periodicIO.getClimberLeftSpeed(), ControlType.kVelocity);
    // m_rightClimberMotorPID.setReference(m_periodicIO.getClimberRightSpeed(), ControlType.kVelocity);

    m_leftClimberMotor.set(m_periodicIO.getClimberLeftSpeed());
    m_rightClimberMotor.set(m_periodicIO.getClimberRightSpeed());

    //SmartDashboard.putNumber("Left speed setpoint:", m_periodicIO.getClimberLeftSpeed());
    //SmartDashboard.putNumber("Left speed:", m_leftClimberEncoder.getVelocity());
    //SmartDashboard.putNumber("Right speed setpoint:", m_periodicIO.getClimberRightSpeed());
    //SmartDashboard.putNumber("Right speed:", m_rightClimberEncoder.getVelocity());
  }

  // - - - - - - - - - - PUBLIC FUNCTIONS - - - - - - - - - -

  public void setBrakeMode() {
    m_leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setCoastMode() {
    m_leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }
  
  public void climb() {
    m_periodicIO.setClimberLeftSpeed(ClimberConstants.kClimberClimbSpeed);
    m_periodicIO.setClimberRightSpeed(ClimberConstants.kClimberClimbSpeed);
  }
  
  public void release() {
    m_periodicIO.setClimberLeftSpeed(ClimberConstants.kClimberReleaseSpeed);
    m_periodicIO.setClimberRightSpeed(ClimberConstants.kClimberReleaseSpeed);
  }
  
  public void tiltLeft() {
    m_periodicIO.setClimberLeftSpeed(ClimberConstants.kClimberReleaseSpeed);
    m_periodicIO.setClimberRightSpeed(0.0);
  }

  public void tiltRight() {
    m_periodicIO.setClimberLeftSpeed(0.0);
    m_periodicIO.setClimberRightSpeed(ClimberConstants.kClimberReleaseSpeed);
  }

  public void stopClimber() {
    m_periodicIO.setClimberLeftSpeed(0.0);
    m_periodicIO.setClimberRightSpeed(0.0);
  }

}