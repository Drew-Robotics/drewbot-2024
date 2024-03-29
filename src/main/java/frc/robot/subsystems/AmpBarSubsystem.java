// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class AmpBarSubsystem extends SubsystemBase {

  private CANSparkMax m_ampBarSpark;
  private SparkPIDController m_pivotPid;
  private AbsoluteEncoder m_pivotEncoder;
  private double commandedPosition = Constants.AmpBarConstants.kAmpBarStowAngleRadians;
  private double currentPositionRad = Constants.AmpBarConstants.kAmpBarStowAngleRadians;

  private static AmpBarSubsystem m_instance;


  /** Creates a new AmpBar. */
  public AmpBarSubsystem() {
    m_ampBarSpark = new CANSparkMax(Constants.AmpBarConstants.kAmpBarPivotSparkCANId, MotorType.kBrushless);

    m_pivotPid = m_ampBarSpark.getPIDController();
    m_pivotEncoder = m_ampBarSpark.getAbsoluteEncoder(Type.kDutyCycle);

    m_pivotPid.setFeedbackDevice(m_pivotEncoder);
    m_pivotEncoder.setPositionConversionFactor(2 * Math.PI); //radians
    m_pivotEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0); //radians per second

    m_pivotPid.setP(Constants.AmpBarConstants.kAmpBarP);
    m_pivotPid.setI(Constants.AmpBarConstants.kAmpBarI);
    m_pivotPid.setD(Constants.AmpBarConstants.kAmpBarD);
    m_pivotPid.setOutputRange(-1, 1);

    m_ampBarSpark.restoreFactoryDefaults();
    m_ampBarSpark.enableVoltageCompensation(12);
    m_ampBarSpark.setIdleMode(IdleMode.kBrake);
    //neo550s blow up if > 20 amps
    m_ampBarSpark.setSmartCurrentLimit(20);
    m_ampBarSpark.setInverted(Constants.AmpBarConstants.kAmpSparkInverted);

    m_ampBarSpark.burnFlash();
  }

  public static AmpBarSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new AmpBarSubsystem();
    }
    return m_instance;
  }

  @Override
  public void periodic() {
    currentPositionRad = m_pivotEncoder.getPosition();
    SmartDashboard.putNumber("Amp Bar Commanded Position", commandedPosition);
    SmartDashboard.putNumber("Amp Bar Current Position", currentPositionRad);
  }

  public boolean isBarAtCommandedPosition() {
    double pivotError = Math.abs(commandedPosition - currentPositionRad);
    if(pivotError < Constants.AmpBarConstants.kAmpBarAllowableError) {
      return true;
    }
    else {
      return false;
    }
  }

  public Command StowAmpBar() {
    return sequence(
      runOnce(() -> {
        commandedPosition = Constants.AmpBarConstants.kAmpBarStowAngleRadians;
        m_pivotPid.setReference(commandedPosition, ControlType.kPosition);
      }),
      waitUntil(() -> {
        return isBarAtCommandedPosition();
      })
    );
  }

  public Command DeployAmpBar() {
    return sequence(
      runOnce(() -> {
        commandedPosition = Constants.AmpBarConstants.kAmpBarOutAngleRadians;
        m_pivotPid.setReference(commandedPosition, ControlType.kPosition);
      }),
      waitUntil(() -> {
        return isBarAtCommandedPosition();
      })
    );
  }
}
