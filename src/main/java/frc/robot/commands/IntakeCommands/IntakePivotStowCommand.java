package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePivotStowCommand extends Command{

  IntakeSubsystem m_intake = IntakeSubsystem.getInstance();

  // Constructor
  public IntakePivotStowCommand(){}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setPivotTarget(IntakeSubsystem.PivotTarget.STOW);
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