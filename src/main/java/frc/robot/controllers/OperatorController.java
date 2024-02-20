package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorController extends CommandXboxController{

  public OperatorController(int port){
    super(port);
  }

  public Trigger getShooterShootTrigger(){
    return leftBumper();
  }

  public Trigger getShooterReverseTrigger(){
    return rightBumper();
  }

  public Trigger getIntakePivotStowTrigger(){
    return povLeft();
  }

  public Trigger getIntakePivotAmpTrigger(){
    return povUp();
  }

  public Trigger getIntakePivotGroundTrigger(){
    return povRight();
  }

  public Trigger getIntakeIntakeTrigger(){
    return rightTrigger(0.5);
  }

  public Trigger getIntakeFeedTrigger(){
    return leftTrigger(0.5);
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
