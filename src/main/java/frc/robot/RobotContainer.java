// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbersState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.PivotState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriverCommands.DriveCommand;
import frc.robot.commands.DriverCommands.DriveStopCommand;
import frc.robot.commands.DriverCommands.DriveTurnToAngleCommand;
import frc.robot.commands.DriverCommands.DriveZeroYawCommand;
import frc.robot.commands.OperatorCommands.IntakeAmpShootCommand;
import frc.robot.commands.OperatorCommands.IntakeDetectNoteCommand;
import frc.robot.commands.OperatorCommands.IntakeDownCommand;
import frc.robot.commands.OperatorCommands.IntakeEjectCommand;
import frc.robot.commands.OperatorCommands.ShootCommands.ShooterRevCommand;
import frc.robot.commands.OperatorCommands.ShootCommands.ShooterShootCommand;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // - - - - - - - - - - FIELDS AND CONSTRUCTORS - - - - - - - - - -

  // The robot's subsystems
  private final DriveSubsystem m_drive = DriveSubsystem.getInstance();
  private final ShooterSubsystem m_shooter = ShooterSubsystem.getInstance();
  private final IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  private final ClimberSubsystem m_climber = ClimberSubsystem.getInstance();

  private final SendableChooser<Command> autoChooser;

  

  DriverController m_driverController = new DriverController(OIConstants.kDriverControllerPort);
  OperatorController m_operatorController = new OperatorController(OIConstants.kOperatorControllerPort);

  /**
   * Constructor.
   */
  private RobotContainer(){
    // Configure the button bindings
    configureDriverCommands();
    configureOperatorCommands();

    autoChooser = AutoBuilder.buildAutoChooser("Default Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);


    // Intake
    NamedCommands.registerCommand("intakeDown", new IntakeDownCommand());
    NamedCommands.registerCommand("intakeEject", new IntakeEjectCommand());
    NamedCommands.registerCommand("intakeAmpShoot", new IntakeAmpShootCommand());

    // Intake Pivot
    NamedCommands.registerCommand("intakePivotGround", IntakeSubsystem.pivotCommand(PivotState.GROUND));
    NamedCommands.registerCommand("intakePivotAmp", IntakeSubsystem.pivotCommand(PivotState.AMP));
    NamedCommands.registerCommand("intakePivotStow", IntakeSubsystem.pivotCommand(PivotState.STOW));

    // Shooter
    NamedCommands.registerCommand("shootSpeakerRev", new ShooterRevCommand(ShooterState.SPEAKER));
    NamedCommands.registerCommand("shootSpeaker", new ShooterShootCommand(ShooterState.SPEAKER));
    NamedCommands.registerCommand("shootAmpRev", new ShooterRevCommand(ShooterState.AMP));
    NamedCommands.registerCommand("shootAmp", new ShooterShootCommand(ShooterState.AMP));
  }

  private static RobotContainer m_instance;

  /**
   * Returns an instance of robot, this is an implementation of the singleton design pattern.
   * @return instance
   */
  public static RobotContainer getInstance(){
    if (m_instance == null){
      m_instance = new RobotContainer();
    }
    return m_instance;
  }
  
  // - - - - - - - - - - PRIVATE FUNCTIONS - - - - - - - - - -

  private void configureDriverCommands(){

    // Configure default commands
    DriveCommand defaultDriveCommand = 
      new DriveCommand(
          m_driverController::getXSpeed,
          m_driverController::getYSpeed,
          m_driverController::getRotation,
          true, true
      );
    
    defaultDriveCommand.addRequirements(m_drive);
    m_drive.setDefaultCommand(defaultDriveCommand);

    // Configure buttons
    // m_driverController.getZeroYawButton().onTrue(new DriveZeroYawCommand());
    SmartDashboard.putData("Zero Yaw", new DriveZeroYawCommand());
    m_driverController.getTurnToZeroButton().whileTrue(new DriveTurnToAngleCommand(0));
    m_driverController.getStopButton().onTrue(new DriveStopCommand());
  }

  private void configureOperatorCommands(){

    // Shooter
    m_operatorController.getShooterSpeakerTrigger()
      .onTrue(new ShooterRevCommand(ShooterState.SPEAKER));

    m_operatorController.getShooterSpeakerTrigger()
      .onFalse(new ShooterShootCommand(ShooterState.SPEAKER));

    m_operatorController.getShooterAmpTrigger()
      .onTrue(new ShooterRevCommand(ShooterState.AMP));

    m_operatorController.getShooterAmpTrigger()
      .onFalse(new ShooterShootCommand(ShooterState.AMP));


    // Intake Amp Shoot
    m_operatorController.getIntakeAmpTrigger()
      .onTrue(new IntakeAmpShootCommand());

    // Intake
    m_operatorController.getIntakeDetectNoteTrigger()
      .onTrue(new IntakeDownCommand());
    
    m_operatorController.getIntakeEjectNoteTrigger()
      .onTrue(new IntakeEjectCommand());


    // Intake Pivot
    m_operatorController.getIntakePivotGroundTrigger()
      .onTrue(IntakeSubsystem.pivotCommand(PivotState.GROUND));
    m_operatorController.getIntakePivotAmpTrigger()
      .onTrue(IntakeSubsystem.pivotCommand(PivotState.AMP));
    m_operatorController.getIntakePivotStowTrigger()
      .onTrue(IntakeSubsystem.pivotCommand(PivotState.STOW));

    // Climber
    SmartDashboard.putData(
      "Zero Climbers", 
      new RunCommand(
        () -> m_climber.climbersSetZero(),
        m_climber
      )
    );

    m_operatorController.getClimberUpTrigger()
      .onTrue(ClimberSubsystem.climberCommand(ClimbersState.UP));
    m_operatorController.getClimberDownTrigger()
      .onTrue(ClimberSubsystem.climberCommand(ClimbersState.DOWN));
  }
  
  // - - - - - - - - - - PUBLIC FUNCTIONS - - - - - - - - - -

  /**
   * Returns the DriverController object.
   * @return The driver controller
   */
  public DriverController getDriverContoller(){
    return m_driverController;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
