package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class DriverController extends CommandXboxController{

    public DriverController(int port){
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
