package frc.robot.commands.OperatorCommands.ShootCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class ShooterShootCommand extends ParallelCommandGroup{
  ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();
  IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  
  public ShooterShootCommand(ShooterState target){
    addCommands(
      Commands.sequence(
        new RunCommand(
          () -> m_intake.setIntakeState(
            target == ShooterState.SPEAKER ? 
              IntakeState.FEED_SPEAKER_SHOOTER : 
              IntakeState.FEED_AMP_SHOOTER
            ),
          m_intake
        ),
        new WaitCommand(1.5),
        new RunCommand(
          () -> m_intake.setIntakeState(IntakeState.NONE),
          m_intake
        )
      ),
      new RunCommand(
        () -> m_shooter.setShooterState(target),
        m_shooter
      ),
      new WaitCommand(1),
      new RunCommand(
        () -> m_shooter.setShooterState(ShooterState.NONE),
        m_shooter
      )
    );
  }
}
