package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDetectNoteCommand extends Command{

  IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  boolean hasNote;

  // Constructor
  public IntakeDetectNoteCommand(){
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setPivotTarget(IntakeSubsystem.PivotState.GROUND);
    m_intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    hasNote = m_intake.getTimeOfFlightRange() < IntakeConstants.kNoteIntakedSensorValue;

    if (hasNote){
      m_intake.setPivotTarget(IntakeSubsystem.PivotState.STOW);
      m_intake.setIntakeState(IntakeSubsystem.IntakeState.NONE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeState(IntakeSubsystem.IntakeState.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasNote;
  }
}