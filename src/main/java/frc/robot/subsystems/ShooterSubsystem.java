// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ShooterSubsystem extends SubsystemBase {

  // - - - - - - - - - - FIELDS AND CONSTRUCTORS - - - - - - - - - -

  private CANSparkFlex m_leftShooterMotor;
  private CANSparkFlex m_rightShooterMotor;

  private SparkPIDController m_leftShooterPID;
  private SparkPIDController m_rightShooterPID;

  private double m_shooterSpeed = 0;
  private ShooterState m_shooterState = ShooterState.NONE;

  // private RelativeEncoder m_leftShooterEncoder;
  // private RelativeEncoder m_rightShooterEncoder;

  private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1000);

  private static ShooterSubsystem m_instance;

  /**
   * Constructor.
   */
  public ShooterSubsystem(){
    m_leftShooterMotor = new CANSparkFlex(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
    m_rightShooterMotor = new CANSparkFlex(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

    m_leftShooterMotor.restoreFactoryDefaults();
    m_rightShooterMotor.restoreFactoryDefaults();

    m_leftShooterPID = m_leftShooterMotor.getPIDController();
    m_rightShooterPID = m_rightShooterMotor.getPIDController();

    m_leftShooterPID.setP(ShooterConstants.kShooterP);
    m_leftShooterPID.setI(ShooterConstants.kShooterI);
    m_leftShooterPID.setD(ShooterConstants.kShooterD);
    m_leftShooterPID.setFF(ShooterConstants.kShooterFF);
    m_leftShooterPID.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);
    
    m_rightShooterPID.setP(ShooterConstants.kShooterP);
    m_rightShooterPID.setI(ShooterConstants.kShooterI);
    m_rightShooterPID.setD(ShooterConstants.kShooterD);
    m_rightShooterPID.setFF(ShooterConstants.kShooterFF);
    m_rightShooterPID.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);

    // There is a crash issue with these.
    // https://www.chiefdelphi.com/t/psa-new-crash-bug-in-revlib-2024-2-2/456242?u=jreneew2
    // m_leftShooterEncoder = m_leftShooterMotor.getEncoder();
    // m_rightShooterEncoder = m_rightShooterMotor.getEncoder();

    m_leftShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    m_rightShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    m_leftShooterMotor.setInverted(true);
    m_rightShooterMotor.setInverted(false);
  }


  /**
   * Returns an instance of robot, this is an implementation of the singleton design pattern.
   * @return instance
   */
  public static ShooterSubsystem getInstance(){
    if (m_instance == null){
      m_instance = new ShooterSubsystem();
    }
    return m_instance;
  }

  public enum ShooterState {
    SPEAKER,
    AMP,
    NONE
  }

  // - - - - - - - - - - GENERIC FUNCTIONS - - - - - - - - - -

  @Override
  public void periodic(){
    m_shooterSpeed = shooterStateToSpeed(m_shooterState);
    double limitedSpeed = m_speedLimiter.calculate(m_shooterSpeed);

    m_leftShooterMotor.set(limitedSpeed);
    m_rightShooterMotor.set(limitedSpeed);

    // SmartDashboard.putNumber("Shooter Speed", limitedSpeed);
  }

  // - - - - - - - - - - PRIVATE FUNCTIONS - - - - - - - - - -

  private double shooterStateToSpeed(ShooterState state){
    switch (state) {
      case SPEAKER:
        return ShooterConstants.kShooterSpeakerSpeed;
      case AMP:
        return ShooterConstants.kShooterAmpSpeed;
      case NONE:
        return 0;
      default:
        return 0;
    }
  }

  // - - - - - - - - - - PUBLIC FUNCTIONS - - - - - - - - - -

  public void setShooterState(ShooterState state) {
    m_shooterState = state;
  }

  public static Command shooterCommand(ShooterState state) {
    return new RunCommand(
      () -> m_instance.setShooterState(state),
      m_instance
    ).withTimeout(0);
  }
  
}