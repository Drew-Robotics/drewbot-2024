// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.controllers.DriverController;

public class DriveTurnToAngleCommand extends Command {
  private final DriveSubsystem m_drive;
  PIDController m_pid;
  double m_target_angle;

  /** Creates a new SetReverseIntakeSpeed. */
  public DriveTurnToAngleCommand(double angle) {
    m_drive = DriveSubsystem.getInstance();

    //m_target_angle = clamp_180(m_drive.getYaw() + angle);
    m_target_angle = angle;

    m_pid = new PIDController(
      DriveConstants.kP,
      DriveConstants.kI,
      DriveConstants.kD
    );

    m_pid.setTolerance(5f);
    //m_pid.enableContinuousInput(-180, 180);

    SmartDashboard.putData("PID Controller", m_pid);
    //LiveWindow.add(m_pid);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // helper functions

  /**
   * returns an equivalent angle from the range -180 to 180.
   * 
   * @param angle the angle to clamp
   * @return the clamped angle
   */
  private double clamp_180(double angle){
    if (angle > 180){
      return clamp_180(angle-360);
    }
    else if (angle < -180){
      return clamp_180(angle+360);
    }
    return angle;
  }

  /**
   * returns the angle needed to get from current_angle to target_angle
   * making use of the clamp_180 function
   * 
   * @param current_angle starting angle
   * @param target_angle the target angle
   * @return the angle difference
   */
  private double angle_dif(double current_angle, double target_angle){
    current_angle = clamp_180(current_angle);
    target_angle = clamp_180(target_angle);

    double difference = clamp_180(target_angle - current_angle);
    return difference;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pid_out = m_pid.calculate(m_target_angle-angle_dif(m_drive.getYaw(), m_target_angle), m_target_angle);
    SmartDashboard.putNumber("pid_out", pid_out);
    SmartDashboard.putNumber("target angle", m_target_angle);
    SmartDashboard.putNumber("current angle", m_target_angle-angle_dif(m_drive.getYaw(), m_target_angle));
    SmartDashboard.putNumber("pid pos error", m_pid.getPositionError());

    // I don't like doing this but it must be done.
    DriverController driverController = RobotContainer.getInstance().getDriverContoller();
    m_drive.drive(driverController.getXSpeed(), driverController.getYSpeed(), -pid_out/100, false, true);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pid.atSetpoint();
  }
}
