package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.OperatorCommands.WaitCommands.IntakeDetectNoteCommand;
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

      new ParallelCommandGroup(
        // RobotContainer.getInstance().rumbleCommand(1, 1),

        new SequentialCommandGroup(
          new WaitCommand(0.5),
          IntakeSubsystem.stateCommand(IntakeState.NONE),
          IntakeSubsystem.pivotCommand(PivotState.STOW)
        )
      )

    );
  }

}
