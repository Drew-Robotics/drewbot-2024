// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.utils.SwerveUtils;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import java.sql.Driver;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class DriveSubsystem extends SubsystemBase {

  // - - - - - - - - - - FIELDS AND CONSTRUCTORS - - - - - - - - - -

  // Create field for the NavX
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kMXP, AHRS.SerialDataType.kProcessedData, (byte) 50);

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      "Front Left");

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      "Front Right");

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      "Rear Left");

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      "Rear Right");
  
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private class WheelRadCharData {
    public double lastGyroYaw = 0;
    public double accumGyroYaw = 0;
    public double[] startWheelPositions = new double[4];
    public double effectiveWheelRadius = 0;
    SlewRateLimiter omegaLimiter = new SlewRateLimiter(1);
  };

  private final double driveRadius = Math.sqrt(Math.pow(DriveConstants.kTrackWidth, 2) + Math.pow(DriveConstants.kWheelBase, 2));

  WheelRadCharData wheelData = new WheelRadCharData();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }
  );

  SwerveDrivePoseEstimator m_PoseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics, 
    Rotation2d.fromDegrees(getAngle()), 
    new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, 
    getPose());

  SendableChooser<Double> m_offsetChooser = new SendableChooser<Double>();

  final StructTopic<Pose2d> visionTopic;
  final StructPublisher<Pose2d> visionPub;

  final StructTopic<Pose2d> robotPoseTopic;
  final StructPublisher<Pose2d> robotPosePub;

  final StructTopic<Pose2d> robotOdomTopic;
  final StructPublisher<Pose2d> robotOdomPub;
  /** 
   * Constructor.
   */
  private DriveSubsystem() {
    m_offsetChooser.addOption("Middle", 0d);
    m_offsetChooser.addOption("Bottom", -60d);
    m_offsetChooser.addOption("Top", 60d);

    m_offsetChooser.setDefaultOption("Middle", 0d);

    visionTopic = NetworkTableInstance.getDefault().getStructTopic("Vision/PoseToAdd", Pose2d.struct);
    visionPub = visionTopic.publish();

    robotPoseTopic = NetworkTableInstance.getDefault().getStructTopic("Vision/RobotPosition", Pose2d.struct);
    robotPosePub = robotPoseTopic.publish();

    robotOdomTopic = NetworkTableInstance.getDefault().getStructTopic("Vision/RobotOdom", Pose2d.struct);
    robotOdomPub = robotOdomTopic.publish();

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
      this::getEstimPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(AutoConstants.kDrivingP, AutoConstants.kDrivingI, AutoConstants.kDrivingD), // Translation PID constants
        new PIDConstants(AutoConstants.kTurningP, AutoConstants.kTurningI, AutoConstants.kTurningD), // Rotation PID constants
        4.5, // Max module speed, in m/s
        Math.sqrt(Math.pow(DriveConstants.kTrackWidth, 2) + Math.pow(DriveConstants.kWheelBase, 2)), // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirement
    );
  }

  private static DriveSubsystem m_instance;

  /**
   * Returns an instance of robot, this is an implementation of the singleton design pattern.
   * @return instance
   */
  public static DriveSubsystem getInstance(){
    if (m_instance == null){
      m_instance = new DriveSubsystem();
    }
    return m_instance;
  }

  // - - - - - - - - - - GENERIC FUNCTIONS - - - - - - - - - -

  @Override
  public void periodic() {
    // Update SmartDashboard with NavX values.
    SmartDashboard.putBoolean("IMU Connected", m_gyro.isConnected());
    SmartDashboard.putNumber("IMU Angle", getAngleClamped());
    SmartDashboard.putData("IMU Angle Offset Chooser", m_offsetChooser);
    robotPosePub.set(m_PoseEstimator.getEstimatedPosition());
    robotOdomPub.set(m_odometry.getPoseMeters());

    if(RobotState.isTeleop()) {
      setAngleAdjustment(m_offsetChooser.getSelected());
    }


    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    m_PoseEstimator.update(
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  // - - - - - - - - - - PUBLIC FUNCTIONS - - - - - - - - - -

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Pose2d getEstimPose() {
    return m_PoseEstimator.getEstimatedPosition();
  }

  public void AddVisionMeasurement(Pose2d poseToAdd, double timestamp, Matrix<N3, N1> stdDevs) {
    Pose2d poseToAdd2;
    poseToAdd2 = new Pose2d(poseToAdd.getTranslation(), poseToAdd.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    visionPub.set(poseToAdd2);
    if(poseToAdd2.getX() < 0 || poseToAdd2.getY() < 0) {
      return;
    }
    if(poseToAdd2.getX() > VisionConstants.LAYOUT.getFieldLength() || poseToAdd2.getY() > VisionConstants.LAYOUT.getFieldWidth()) {
      return;
    }
    m_PoseEstimator.addVisionMeasurement(poseToAdd2, timestamp, stdDevs);
  }


  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Gets the swerve ModuleStates.
   * 
   * @return The SwerveModule states.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
      };
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * empty javadoc.
   * 
   * @return
   */
  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the robot yaw from the NavX.
   * 
   * @return Returns the current yaw value in degrees from -180 to 180.
   */
  public double getAngleClamped() {
    double angle = getAngle() + 180;

    while (angle > 360){angle -= 360;}
    while (angle < 0){angle += 360;}

    return angle - 180; 
  }


  public double getAngle() {
    return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
   return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns if the robot is moving or not.
   * 
   * @return is moving
   */
  public boolean isMoving(){
    return m_gyro.isMoving();
  }  
  
  /**
   * Sets the angle adjustment of the gyroscope.
   */
  public void setAngleAdjustment(double adjustment){
    m_gyro.setAngleAdjustment(adjustment);
  }
  
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    //DREW
    m_gyro.zeroYaw();
    m_gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      pose
    );
    m_PoseEstimator.resetPosition(
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
      },
      pose
    );
  }

  /**
   * Sets the yaw of the gyroscope to zero.
   */
  public void zeroYaw() {
    m_gyro.zeroYaw(); 
  }

  /** 
   * Resets the drive encoders to currently read a position of 0. 
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * empty javadoc
   * 
   * @param robotRelativeSpeeds
   */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot      Angular rate of the robot input.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded;
    double ySpeedCommanded;

    xSpeed *= DriveConstants.kTranslationScalar;
    ySpeed *= DriveConstants.kTranslationScalar;
    rot *= DriveConstants.kRotationScalar;

    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putNumber("Rotation", rot);

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));


      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getAngleClamped() - 360))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  SysIdRoutine steerRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism((voltage) -> m_frontLeft.setSteerToVoltage(voltage.in(Volts)), null, this)
  );

  SysIdRoutine driveRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism((voltage) -> {
      double voltageToSend = voltage.in(Volts);
      m_frontLeft.setDriveToVoltage(voltageToSend);
      m_frontRight.setDriveToVoltage(voltageToSend);
      m_rearLeft.setDriveToVoltage(voltageToSend);
      m_rearRight.setDriveToVoltage(voltageToSend);
    }, null, this)
  );

    // The methods below return Command objects
  public Command steerQFwd() {
    return steerRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }
  public Command steerQRev() {
    return steerRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }
  public Command steerDFwd() {
    return steerRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }
  public Command steerDRev() {
    return steerRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public Command driveQFwd() {
    return driveRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }
  public Command driveQRev() {
    return driveRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }
  public Command driveDFwd() {
    return driveRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }
  public Command driveDRev() {
    return driveRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public Command wheelRad(boolean fwd) {
    return Commands.sequence(
      Commands.runOnce(() -> {
        wheelData.lastGyroYaw = Math.toRadians(getAngle());
        wheelData.accumGyroYaw = 0;
        wheelData.startWheelPositions[0] = m_frontLeft.getDriveShaftRadians();
        wheelData.startWheelPositions[1] = m_frontRight.getDriveShaftRadians();
        wheelData.startWheelPositions[2] = m_rearLeft.getDriveShaftRadians();
        wheelData.startWheelPositions[3] = m_rearRight.getDriveShaftRadians();
        wheelData.omegaLimiter.reset(0);
        wheelData.effectiveWheelRadius = 0;
      }, this),
      Commands.runEnd(
      () -> {
        double dirMulti = 1.0;
        if(!fwd) {
          dirMulti = -1.0;
        }
        double currentYawRad = Math.toRadians(getAngle());
        drive(0, 0, wheelData.omegaLimiter.calculate(1 * dirMulti), false, false);
        wheelData.accumGyroYaw += MathUtil.angleModulus(currentYawRad - wheelData.lastGyroYaw);
        wheelData.lastGyroYaw = currentYawRad;
        double avgWheelPos = 0;
        double[] currentPos = new double[4];
        currentPos[0] = m_frontLeft.getDriveShaftRadians();
        currentPos[1] = m_frontRight.getDriveShaftRadians();
        currentPos[2] = m_rearLeft.getDriveShaftRadians();
        currentPos[3] = m_rearRight.getDriveShaftRadians();
        for(int i = 0; i < 4; i++) {
          avgWheelPos += Math.abs(currentPos[i] - wheelData.startWheelPositions[i]);
        }
        avgWheelPos /= 4.0;
        wheelData.effectiveWheelRadius = (wheelData.accumGyroYaw * driveRadius) / avgWheelPos;
      }, 
      () -> {
        drive(0, 0, 0, false, false);
        System.out.println("WHEEL RADIUS METERS: " + wheelData.effectiveWheelRadius + "\n\n\n\n\n\n");
      }, this)
    ).withName("Wheel Radius Cmd");
  }
}