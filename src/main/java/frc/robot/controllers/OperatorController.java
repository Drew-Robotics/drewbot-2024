package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorController extends XboxController{

  public OperatorController(int port){
    super(port);
  }

  public Button getShootButton(){
    return Button.kRightBumper;
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
