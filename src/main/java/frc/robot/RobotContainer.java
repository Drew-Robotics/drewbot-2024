// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;


import frc.robot.controllers.DriverController;
import frc.robot.controllers.OperatorController;
import frc.robot.commands.DriveCommands.DriveDriveCommand;
import frc.robot.commands.DriveCommands.DriveTurnToAngleCommand;
import frc.robot.commands.DriveCommands.DriveZeroYawCommand;
import frc.robot.commands.IntakeCommands.IntakeDetectNoteCommand;
import frc.robot.commands.IntakeCommands.IntakeBasicCommands.IntakePivotAmpCommand;
import frc.robot.commands.IntakeCommands.IntakeBasicCommands.IntakePivotGroundCommand;
import frc.robot.commands.IntakeCommands.IntakeBasicCommands.IntakePivotStowCommand;
import frc.robot.commands.IntakeCommands.IntakeBasicCommands.IntakeStateEjectCommand;
import frc.robot.commands.IntakeCommands.IntakeBasicCommands.IntakeStateFeedCommand;
import frc.robot.commands.IntakeCommands.IntakeBasicCommands.IntakeStateIntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeBasicCommands.IntakeStateStopCommand;
import frc.robot.commands.ShooterCommands.ShooterBasicCommands.ShooterReverseCommand;
import frc.robot.commands.ShooterCommands.ShooterBasicCommands.ShooterSpeakerShootCommand;
import frc.robot.commands.ShooterCommands.ShooterBasicCommands.ShooterStopCommand;
import frc.robot.commands.ShooterCommands.ShooterBasicCommands.ShooterAmpShootCommand;
import frc.robot.commands.DriveCommands.DriveStopCommand;
import frc.robot.commands.ClimberCommands.ClimberClimbCommand;
import frc.robot.commands.ClimberCommands.ClimberReleaseCommand;
import frc.robot.commands.ClimberCommands.ClimberTiltLeftCommand;
import frc.robot.commands.ClimberCommands.ClimberTiltRightCommand;

import java.util.List;

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
  

  DriverController m_driverController = new DriverController(OIConstants.kDriverControllerPort);
  OperatorController m_operatorController = new OperatorController(OIConstants.kOperatorControllerPort);

  /**
   * Constructor.
   */
  private RobotContainer(){
    // Configure the button bindings
    configureDriverCommands();
    configureOperatorCommands();
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
    DriveDriveCommand defaultDriveCommand = 
      new DriveDriveCommand(
          m_driverController::getXSpeed,
          m_driverController::getYSpeed,
          m_driverController::getRotation,
          true, true
      );
    
    defaultDriveCommand.addRequirements(m_drive);
    m_drive.setDefaultCommand(defaultDriveCommand);

    // Configure buttons
    m_driverController.getZeroYawButton().whileTrue(new DriveTurnToAngleCommand(0));
    m_driverController.getTurnToZeroButton().onTrue(new DriveZeroYawCommand());
    m_driverController.getStopButton().onTrue(new DriveStopCommand());
  }

  private void configureOperatorCommands(){
    // Shooter
    m_operatorController.getShooterSpeakerTrigger()
      .onTrue(Commands.parallel(
        new ShooterSpeakerShootCommand(),
        new IntakePivotStowCommand()
      ));
    m_operatorController.getShooterSpeakerTrigger()
      .onFalse(Commands.parallel(
        new IntakeStateFeedCommand().withTimeout(1),
        new ShooterSpeakerShootCommand().withTimeout(1)
      ));

    // Intake
    m_operatorController.getIntakeDetectNoteTrigger().onTrue(new IntakeDetectNoteCommand());

    m_operatorController.getIntakePivotGroundTrigger().onTrue(new IntakePivotGroundCommand());
    m_operatorController.getIntakePivotAmpTrigger().onTrue(new IntakePivotAmpCommand());
    m_operatorController.getIntakePivotStowTrigger().onTrue(new IntakePivotStowCommand());

    // Climber
    m_operatorController.getClimberClimbTrigger().whileTrue(new ClimberClimbCommand());
    m_operatorController.getClimberReleaseTrigger().whileTrue(new ClimberReleaseCommand());
    m_operatorController.getClimberTiltLeftTrigger().whileTrue(new ClimberTiltLeftCommand());
    m_operatorController.getClimberTiltRightTrigger().whileTrue(new ClimberTiltRightCommand());
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
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_drive::setModuleStates,
        m_drive);

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_drive.drive(0, 0, 0, false, false));
  }
}
