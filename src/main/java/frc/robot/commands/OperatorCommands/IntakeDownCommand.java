package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.OperatorCommands.WaitCommands.IntakeDetectNoteCommand;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.PivotState;


public class IntakeDownCommand extends SequentialCommandGroup {
  IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  DriverController m_driverController = DriverController.getIntance();
  OperatorController m_operatorController = OperatorController.getIntance();


  public IntakeDownCommand() {
    addCommands(
      IntakeSubsystem.pivotCommand(PivotState.GROUND),
      IntakeSubsystem.stateCommand(IntakeState.INTAKE),
      new IntakeDetectNoteCommand(),

      new ParallelCommandGroup(
        m_driverController.intakeRumbleCommand().withTimeout(0.3),
        m_operatorController.intakeRumbleCommand().withTimeout(0.3),

        new SequentialCommandGroup(
          new WaitCommand(0.25),
          IntakeSubsystem.stateCommand(IntakeState.NONE),
          IntakeSubsystem.pivotCommand(PivotState.STOW)
        )
      )

    );
  }

}
