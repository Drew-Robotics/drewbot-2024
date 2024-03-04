package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class EverythingController extends CommandXboxController{

  public EverythingController(int port){
    super(port);
  }

  // Drive
  public double getXSpeed(){
    return -MathUtil.applyDeadband(this.getLeftY(), OIConstants.kDriveDeadband);
  }

  public double getYSpeed(){
    return -MathUtil.applyDeadband(this.getLeftX(), OIConstants.kDriveDeadband);
  }

  public double getRotation(){
    return -MathUtil.applyDeadband(this.getRightX(), OIConstants.kDriveDeadband) * DriveConstants.RotationCoeff;
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
}
