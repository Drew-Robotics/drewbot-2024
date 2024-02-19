package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
  private final PIDController m_pivotPID = new PIDController(IntakeConstants.kPivotP, IntakeConstants.kPivotI, IntakeConstants.kPivotD);
  
  private static Intake m_instance;
  
  public static Intake getInstance() {
    if (m_instance = null) {
      m_instance = new Intake();
    }
    return m_instance;
  }

  private CANSparkMax m_intakeMotor;
  private CANSparkMax m_pivotMotor;

  private Intake() {
    // Intake Motor
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast); // Maybe have to change this later?
    // Pivot Motor
    m_pivotMotor = new CANSparkMax(IntakeConstants.kPivotMotorID)
    
  }
}
