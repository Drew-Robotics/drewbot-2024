package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class DriverController extends Controller{

  public static DriverController m_intance;

  public static DriverController getIntance(){
    if (m_intance == null){
        m_intance = new DriverController(OIConstants.kDriverControllerPort);
    }
    return m_intance;
  }

  private DriverController(int port){
    super(port);
  }

  public double getXSpeed(){
    return -MathUtil.applyDeadband(this.getLeftY(), OIConstants.kDriveDeadband);
  }

  public double getYSpeed(){
    return -MathUtil.applyDeadband(this.getLeftX(), OIConstants.kDriveDeadband);
  }

  public double getRotation(){
    return -MathUtil.applyDeadband(this.getRightX(), OIConstants.kDriveDeadband) * DriveConstants.RotationCoeff;
  }

  public Trigger getTurnToZeroButton(){
    return y();
  }

  public Trigger getStopButton(){
    return rightBumper();
  }
}
