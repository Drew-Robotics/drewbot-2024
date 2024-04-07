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
      Commands.parallel(
        Commands.sequence(
          IntakeSubsystem.stateCommand(
            target == ShooterState.SPEAKER ? 
              IntakeState.FEED_SPEAKER_SHOOTER : 
              IntakeState.FEED_AMP_SHOOTER
          ),
          new WaitCommand(0.4),
          IntakeSubsystem.stateCommand(IntakeState.NONE)
        ),
        Commands.sequence(
          ShooterSubsystem.shooterCommand(target),
          new WaitCommand(0.4),
          ShooterSubsystem.shooterCommand(ShooterState.NONE)
        )
      )
    );
  }
}
