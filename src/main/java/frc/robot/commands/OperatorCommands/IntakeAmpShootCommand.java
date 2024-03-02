package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.PivotState;

public class IntakeAmpShootCommand extends SequentialCommandGroup{
  IntakeSubsystem m_intake = IntakeSubsystem.getInstance();


  public IntakeAmpShootCommand(){
    addCommands(
      new RunCommand(
        () -> m_intake.setPivotTarget(PivotState.AMP),
        m_intake
      ),
      // really dont know if this will work might cause a lot of problem beware
      new RunCommand(
        () -> {
          while (m_intake.getPivotState() != PivotState.AMP){}
          return;
        }, 
        m_intake
      ),
      new RunCommand(
        () -> m_intake.setIntakeState(IntakeState.AMP),
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
