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
      IntakeSubsystem.pivotCommand(PivotState.GROUND),
      IntakeSubsystem.stateCommand(IntakeState.INTAKE),
      new IntakeDetectNoteCommand(),
      new WaitCommand(0.1),
      IntakeSubsystem.stateCommand(IntakeState.NONE),
      IntakeSubsystem.pivotCommand(PivotState.STOW)

    );
  }

}
