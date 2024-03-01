package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorController extends CommandXboxController{

  public OperatorController(int port){
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
    return a();
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
