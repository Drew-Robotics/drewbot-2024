package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorController extends CommandXboxController{

  public OperatorController(int port){
    super(port);
  }

  // Shooter
  public Trigger getShooterShootTrigger(){
    return leftBumper();
  }

  public Trigger getShooterReverseTrigger(){
    return rightBumper();
  }

  // Intake Pivot
  public Trigger getIntakePivotStowTrigger(){
    return povLeft();
  }

  public Trigger getIntakePivotAmpTrigger(){
    return povUp();
  }

  public Trigger getIntakePivotGroundTrigger(){
    return povRight();
  }

  // Intake State
  public Trigger getIntakeIntakeTrigger(){
    return rightTrigger(0.5);
  }

  public Trigger getIntakeFeedTrigger(){
    return leftTrigger(0.5);
  }

  // Climber
  public Trigger getClimberClimbTrigger(){
    return y();
  }

  public Trigger getClimberReleaseTrigger(){
    return a();
  }

  public Trigger getClimberTiltLeftTrigger(){
    return x();
  }

  public Trigger getClimberTiltRightTrigger(){
    return b();
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
