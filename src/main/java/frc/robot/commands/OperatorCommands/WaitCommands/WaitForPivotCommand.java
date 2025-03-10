package frc.robot.commands.OperatorCommands.WaitCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotState;

// https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/FunctionalCommand.html

public class WaitForPivotCommand extends Command{

  IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  PivotState waitForState;

  // Constructor
  public WaitForPivotCommand(PivotState state){
    addRequirements(m_intake);
    waitForState = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_intake.getPivotState() == waitForState);
  }
}