// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriverCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveZeroYawCommand extends Command {
  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
  double m_offSet;

  public DriveZeroYawCommand(){
    addRequirements(m_drive);
    m_offSet = 0;
  }

  public DriveZeroYawCommand(double offSet){
    addRequirements(m_drive);
    m_offSet = offSet;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.zeroYaw();
    m_drive.setAngleAdjustment(m_offSet);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
