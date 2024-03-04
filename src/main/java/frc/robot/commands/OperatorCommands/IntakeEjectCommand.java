package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.OperatorCommands.WaitCommands.WaitForPivotCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.PivotState;;

public class IntakeEjectCommand extends SequentialCommandGroup{

  IntakeSubsystem m_intake = IntakeSubsystem.getInstance();

  public IntakeEjectCommand(){
    addCommands(
      IntakeSubsystem.pivotCommand(PivotState.GROUND),
      new WaitForPivotCommand(PivotState.GROUND),
      IntakeSubsystem.stateCommand(IntakeState.EJECT),
      new WaitCommand(1),
      IntakeSubsystem.stateCommand(IntakeState.NONE),
      IntakeSubsystem.pivotCommand(PivotState.GROUND)
    );
  }
}