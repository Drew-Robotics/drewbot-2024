package frc.robot.commands.OperatorCommands.ShootCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotState;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class ShooterRevCommand extends SequentialCommandGroup{
  ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();
  
  public ShooterRevCommand(ShooterState target){
    addCommands(
      ShooterSubsystem.shooterCommand(target)
    );
  }
}
