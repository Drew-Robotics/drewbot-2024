package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;


public class DriveDriveCommand extends Command{
  DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_ySpeed;
  private DoubleSupplier m_rot;

  private boolean m_fieldRelative;
  private boolean m_rateLimit;

  public DriveDriveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, boolean fieldRelative, boolean rateLimit){
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rot = rot;

    m_fieldRelative = fieldRelative;
    m_rateLimit = rateLimit;
    
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(
      m_xSpeed.getAsDouble(), 
      m_ySpeed.getAsDouble(), 
      m_rot.getAsDouble(), 
      m_fieldRelative, m_rateLimit
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
