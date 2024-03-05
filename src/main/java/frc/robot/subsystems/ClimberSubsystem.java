package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  // - - - - - - - - - - FIELDS AND CONSTRUCTORS - - - - - - - - - -

  private static ClimberSubsystem m_instance;

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

  private ClimbersState m_climbersState = ClimbersState.DOWN;
  private double m_leftClimberTargetPos = 0;
  private double m_rightClimberTargetPos = 0;

  // Defining Motor Encoders
  private RelativeEncoder m_leftClimberEncoder;
  private RelativeEncoder m_rightClimberEncoder;

  private double m_leftClimberZeroPos = 0;
  private double m_rightClimberZeroPos = 0;

  private ClimberSubsystem() {
    
    // Setting motor type/ID
    m_leftClimberMotor = new CANSparkMax(ClimberConstants.kLeftClimberMotorID, MotorType.kBrushless);
    m_rightClimberMotor = new CANSparkMax(ClimberConstants.kRightClimberMotorID, MotorType.kBrushless);

    // Set Inverted
    m_leftClimberMotor.setInverted(ClimberConstants.kLeftClimberMotorInverted);
    m_rightClimberMotor.setInverted(ClimberConstants.kRightClimberMotorInverted);

    // Climber Encoder
    m_leftClimberEncoder = m_leftClimberMotor.getEncoder();
    m_rightClimberEncoder = m_rightClimberMotor.getEncoder();

    // Idle mode 
    m_leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    climbersSetZero();
  }

  public enum ClimbersState { 
    UP,
    DOWN
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

    m_leftClimberTargetPos = climberStateToPositions(m_climbersState)[0];
    m_rightClimberTargetPos = climberStateToPositions(m_climbersState)[1];

    double leftSpeed = m_leftClimberMotorPID.calculate(
      getLeftEncoderPos(),
      m_leftClimberTargetPos
    )/20;

    double rightSpeed = m_rightClimberMotorPID.calculate(
      getRightEncoderPos(),
      m_rightClimberTargetPos
    )/20;

    m_leftClimberMotor.set(leftSpeed);
    m_rightClimberMotor.set(rightSpeed);

    // SmartDashboard.putNumber("Climber Left Target", m_leftClimberTargetPos);
    // SmartDashboard.putNumber("Climber Right Target", m_rightClimberTargetPos);

    // SmartDashboard.putNumber("Climber Left Encoder", getLeftEncoderPos());
    // SmartDashboard.putNumber("Climber Right Encoder", getRightEncoderPos());

    // SmartDashboard.putNumber("Climber Left Speed", leftSpeed);
    // SmartDashboard.putNumber("Climber Right Speed", rightSpeed);
  }

  // - - - - - - - - - - PRIVATE FUNCTIONS - - - - - - - - - -

  private double getLeftEncoderPos(){
    return m_leftClimberEncoder.getPosition() + m_leftClimberZeroPos;
  }

  private double getRightEncoderPos(){
    return m_rightClimberEncoder.getPosition() + m_rightClimberZeroPos;
  }

  private double[] climberStateToPositions(ClimbersState state) { 
    switch (state) {
      case UP:
        return new double[]{
          ClimberConstants.kRotationsToUpLeft,
          ClimberConstants.kRotationsToUpRight
        };
      case DOWN:
        return new double[]{
          0,
          0
        };
      default:
        return new double[]{
          0,
          0
        };
    }
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
  
  public void setClimbersState(ClimbersState state){
    m_climbersState = state;
  }

  public ClimbersState getClimberState(){
    return m_climbersState;
  }

  public void climbersSetZero() {
    m_leftClimberZeroPos = -m_leftClimberEncoder.getPosition();
    m_rightClimberZeroPos = -m_rightClimberEncoder.getPosition();
  }

  // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/InstantCommand.html
  public static Command climberCommand(ClimbersState state){
    return new RunCommand(
      () -> m_instance.setClimbersState(state),
      m_instance
    ).withTimeout(0);
  }
}