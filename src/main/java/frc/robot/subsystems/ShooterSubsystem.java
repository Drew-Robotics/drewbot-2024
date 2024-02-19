// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

  private RelativeEncoder m_leftShooterEncoder;
  private RelativeEncoder m_rightShooterEncoder;

  private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1000);

  private static ShooterSubsystem m_instance;
  
  private PeriodicIO m_periodicIO;
    public PeriodicIO getpPeriodicIO(){return m_periodicIO;}

  /**
   * Constructor.
   */
  public ShooterSubsystem(){
    m_leftShooterMotor = new CANSparkFlex(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
    m_rightShooterMotor = new CANSparkFlex(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

    m_leftShooterMotor.restoreFactoryDefaults();
    m_rightShooterMotor.restoreFactoryDefaults();

    m_leftShooterPID = m_leftShooterMotor.getPIDController();

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

    m_leftShooterEncoder = m_leftShooterMotor.getEncoder();
    m_rightShooterEncoder = m_leftShooterMotor.getEncoder();

    m_leftShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    m_rightShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    m_leftShooterMotor.setInverted(true);
    m_leftShooterMotor.setInverted(false);

    m_periodicIO = new PeriodicIO();
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

  private static class PeriodicIO{
    private double shooterRpm = 0;

    public double getShooterRpm(){
      return shooterRpm;
    }
    public void setShooterRpm(double rpm){
      shooterRpm = rpm;
    }
  }

  // - - - - - - - - - - GENERIC FUNCTIONS - - - - - - - - - -

  @Override
  public void periodic(){
    double limitedSpeed = m_speedLimiter.calculate(m_periodicIO.getShooterRpm());
    m_leftShooterPID.setReference(limitedSpeed, ControlType.kVelocity);
    m_rightShooterPID.setReference(limitedSpeed, ControlType.kVelocity);
  }


  // - - - - - - - - - - PUBLIC FUNCTIONS - - - - - - - - - -

  public void setSpeed(double rpm){
    m_periodicIO.setShooterRpm(rpm);
  }

  public void shoot(){
    m_periodicIO.setShooterRpm(ShooterConstants.kShooterShootRPM);
  }

  public void stop(){
    m_periodicIO.setShooterRpm(0);
  }
  
}