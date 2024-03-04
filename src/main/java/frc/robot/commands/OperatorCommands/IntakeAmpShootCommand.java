package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.OperatorCommands.WaitCommands.WaitForPivotCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.PivotState;

public class IntakeAmpShootCommand extends SequentialCommandGroup{
  IntakeSubsystem m_intake = IntakeSubsystem.getInstance();


  public IntakeAmpShootCommand(){
    addCommands(
      IntakeSubsystem.pivotCommand(PivotState.AMP),
      new WaitForPivotCommand(PivotState.AMP),
      IntakeSubsystem.stateCommand(IntakeState.AMP),
      new WaitCommand(1),
      IntakeSubsystem.stateCommand(IntakeState.NONE),
      IntakeSubsystem.pivotCommand(PivotState.STOW)
    );
  }
  
}
