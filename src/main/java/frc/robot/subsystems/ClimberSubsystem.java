package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

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
  private PIDController m_leftClimberMotorPID = new PIDController(
    ClimberConstants.kClimberP, 
    ClimberConstants.kClimberI,
    ClimberConstants.kClimberD
  );

  private PIDController m_rightClimberMotorPID = new PIDController(
    ClimberConstants.kClimberP, 
    ClimberConstants.kClimberI,
    ClimberConstants.kClimberD
  );

  // Defining Motor Encoders
  private RelativeEncoder m_leftClimberEncoder;
  private RelativeEncoder m_rightClimberEncoder;

  private double m_leftClimberZeroPos = 0;
  private double m_rightClimberZeroPos = 0;

  private ClimberSubsystem() {
    
    m_periodicIO = new PeriodicIO();
    
    // Setting motor type/ID
    m_leftClimberMotor = new CANSparkMax(ClimberConstants.kLeftClimberMotorID, MotorType.kBrushless);
    m_rightClimberMotor = new CANSparkMax(ClimberConstants.kRightClimberMotorID, MotorType.kBrushless);

    // Set inverted
    m_leftClimberMotor.setInverted(ClimberConstants.kLeftClimberMotorInverted);
    m_rightClimberMotor.setInverted(ClimberConstants.kRightClimberMotorInverted);

    // Left climber encoder
    m_leftClimberEncoder = m_leftClimberMotor.getEncoder();
    // m_leftClimberEncoder.setPositionConversionFactor(ClimberConstants.kClimberGearRatio);
    // m_leftClimberEncoder.setVelocityConversionFactor(ClimberConstants.kClimberGearRatio);

    // Right climber encoder
    m_rightClimberEncoder = m_rightClimberMotor.getEncoder();
    // m_rightClimberEncoder.setPositionConversionFactor(ClimberConstants.kClimberGearRatio);
    // m_rightClimberEncoder.setPositionConversionFactor(ClimberConstants.kClimberGearRatio);

    // Idle mode 
    m_leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Inverting motors
    m_leftClimberMotor.setInverted(false);
    m_rightClimberMotor.setInverted(true);
  }


  private static class PeriodicIO {
    private double climberRightPosition = 0.0;
    private double climberLeftPosition = 0.0;

    public double getClimberRightPosition(){
      return climberRightPosition;
    }
    public void setClimberRightPosition(double position){
      climberRightPosition = position;
    }

    public double getClimberLeftPosition(){
      return climberLeftPosition;
    }
    public void setClimberLeftPosition(double position){
      climberLeftPosition = position;
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
    //m_leftClimberMotorPID.setReference(m_periodicIO.getClimberLeftPosition(), ControlType.kPosition);
    //m_rightClimberMotorPID.setReference(m_periodicIO.getClimberRightPosition(), ControlType.kPosition);

    double leftSpeed = m_leftClimberMotorPID.calculate(
      m_leftClimberEncoder.getPosition(),
      m_periodicIO.getClimberLeftPosition()
    );

    double rightSpeed = m_rightClimberMotorPID.calculate(
      m_rightClimberEncoder.getPosition(),
      m_periodicIO.getClimberRightPosition()
    );

    m_leftClimberMotor.set(leftSpeed);
    m_leftClimberMotor.set(rightSpeed);

    //SmartDashboard.putNumber("Left Speed Setpoint", m_periodicIO.getClimberLeftSpeed());
    //SmartDashboard.putNumber("Left Speed:", m_leftClimberEncoder.getVelocity());
    //SmartDashboard.putNumber("Right Speed setpoint", m_periodicIO.getClimberRightSpeed());

    SmartDashboard.putNumber("Climber Left Target", m_periodicIO.getClimberLeftPosition());
    SmartDashboard.putNumber("Climber Right Target", m_periodicIO.getClimberRightPosition());

    SmartDashboard.putNumber("Climber Left Encoder", m_leftClimberEncoder.getPosition());
    SmartDashboard.putNumber("Climber Right Encoder", m_rightClimberEncoder.getPosition());
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
  
  public void climbersUp() {
    m_periodicIO.setClimberLeftPosition(ClimberConstants.kRotationsToUpLeft + m_leftClimberZeroPos);
    m_periodicIO.setClimberRightPosition(ClimberConstants.kRotationsToUpRight + m_leftClimberZeroPos);
  }
  
  public void climbersDown() {
    m_periodicIO.setClimberLeftPosition(m_leftClimberZeroPos);
    m_periodicIO.setClimberRightPosition(m_rightClimberZeroPos);
  }

  public void climbersSetZero() {
    m_leftClimberZeroPos = m_leftClimberEncoder.getPosition();
    m_rightClimberZeroPos = m_rightClimberEncoder.getPosition();
  }

  public void tiltLeft() {
    m_periodicIO.setClimberLeftPosition(m_periodicIO.getClimberLeftPosition() - ClimberConstants.kTiltRPM * 0.02/60);
  }

  public void tiltRight() {
    m_periodicIO.setClimberRightPosition(m_periodicIO.getClimberRightPosition() - ClimberConstants.kTiltRPM * 0.02/60);
  }


}