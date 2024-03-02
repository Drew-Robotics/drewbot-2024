package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.PivotState;


public class IntakeDownCommand extends SequentialCommandGroup {
  IntakeSubsystem m_intake = IntakeSubsystem.getInstance();


  public IntakeDownCommand() {
    addCommands(
      new RunCommand(
        () -> m_intake.setPivotTarget(PivotState.GROUND),
        m_intake
      ),
      new RunCommand(
        () -> m_intake.setIntakeState(IntakeState.INTAKE),
        m_intake
      ),
      new IntakeDetectNoteCommand(),
      new WaitCommand(0.1),
      new RunCommand(
        () -> m_intake.setIntakeState(IntakeState.NONE),
        m_intake
      ),
      new RunCommand(
        () -> m_intake.setPivotTarget(PivotState.STOW),
        m_intake
      )
    );
  }

}
