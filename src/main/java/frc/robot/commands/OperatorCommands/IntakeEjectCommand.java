package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.PivotState;;

public class IntakeEjectCommand extends SequentialCommandGroup{

  IntakeSubsystem m_intake = IntakeSubsystem.getInstance();

  public IntakeEjectCommand(){
    addCommands(
      new RunCommand(
        () -> m_intake.setPivotTarget(PivotState.GROUND),
        m_intake
      ),
      // could cause problems idk if this will work beware
      new RunCommand(
        () -> {
          while (m_intake.getPivotState() != PivotState.GROUND){}
          return;
        },
        m_intake
      ),
      new RunCommand(
        () -> m_intake.setIntakeState(IntakeState.EJECT),
        m_intake
      ),
      new WaitCommand(1),
      new RunCommand(
        () -> m_intake.setIntakeState(IntakeState.NONE),
        m_intake
      )
    );
  }
}