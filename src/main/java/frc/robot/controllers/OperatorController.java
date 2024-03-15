package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;

public class OperatorController extends Controller{

  public static OperatorController m_intance;

  public static OperatorController getIntance(){
    if (m_intance == null){
        m_intance = new OperatorController(OIConstants.kOperatorControllerPort);
    }
    return m_intance;
  }

  private OperatorController(int port){
    super(port);
  }

  // Intake
  public Trigger getIntakeDetectNoteTrigger(){
    return rightBumper();
  }
  public Trigger getIntakeEjectNoteTrigger(){
    return leftBumper();
  }

  public Trigger getIntakeAmpTrigger(){
    return leftTrigger(0.5);
  }

  public Trigger getIntakeHoldTrigger(){
    return leftBumper();
  }

  // Shooter
  public Trigger getShooterSpeakerTrigger(){
    return rightTrigger(0.5);
  }
  public Trigger getShooterAmpTrigger(){
    return leftTrigger(0.5);
  }

  // Intake Pivot
  public Trigger getIntakePivotStowTrigger(){
    return x();
  }

  public Trigger getIntakePivotAmpTrigger(){
    return y();
  }

  public Trigger getIntakePivotGroundTrigger(){
    return b();
  }


  // Climber
  public Trigger getClimberUpTrigger(){
    return povUp();
  }

  public Trigger getClimberDownTrigger(){
    return povDown();
  }

  public Trigger getClimberLeftTrigger(){
    return povLeft();
  }

  public Trigger getClimberRightTrigger(){
    return povRight();
  }

  /*
   * INTAKE
   * dpad: l-shooter, u-up, r-intake, d-source
   * triggers: l-out, r-in
   * 
   * Shooter
   * bumpers: l-out, r-in
   */
  
}
