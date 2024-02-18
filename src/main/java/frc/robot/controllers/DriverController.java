package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OIConstants;;

public class DriverController extends XboxController {

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
        return -MathUtil.applyDeadband(this.getRightX(), OIConstants.kDriveDeadband);
    }

    public Button getZeroYawButton(){
        return Button.kB;
    }

    public Button getTurnToZeroButton(){
        return Button.kY;
    }
}
