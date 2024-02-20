package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorController extends XboxController{

  public OperatorController(int port){
    super(port);
  }

  public Button getShooterShootButton(){
    return Button.kLeftBumper;
  }

  public Button getShooterReverseButton(){
    return Button.kRightBumper;
  }

  public int getIntakePivotStowPOVNumber() {
    return 270;
  }

  public int getIntakePivotAmpPOVNumber() {
    return 0;
  }

  public int getIntakePivotGroundPOVNumber() {
    return 902;
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
