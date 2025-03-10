// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 8;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = true;

    public static final double kTranslationScalar = 0.3;
    public static final double kRotationScalar = 0.3;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1.1 / kDriveWheelFreeSpeedRps;
    
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kSysIdControllerPort = 3;

    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final double kDrivingP = 3;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0.3;
    public static final double kTurningD = 0.1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {

    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ShooterConstants{

    public static final int kLeftShooterCanId = 20;
    public static final int kRightShooterCanId = 21;

    public static final double kShooterP = 0.1;
    public static final double kShooterI = 0.1;
    public static final double kShooterD = 0.1;
    public static final double kShooterFF = 0;

    public static final double kShooterMinOutput = 0;
    public static final double kShooterMaxOutput = 1;

    public static final double kShooterSpeakerSpeed = 1;
    public static final double kShooterAmpSpeed = 0.25;
    public static final double kShooterReverseSpeed = 0.1;
    
  }

  public static final class IntakeConstants {

    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 10;
    public static final double kA = 0;

    public static final double kMaxVelocityRps = 1.0;
    public static final double kMaxAccelerationRpsps = 100.0;

    public static final double kSlowIntake = 5;

    public static final double kPivotP = 45 / kSlowIntake;
    public static final double kPivotI = 25 / kSlowIntake;
    public static final double kPivotD = 3.5 / kSlowIntake;

    public static final double kPivotPIDTolerance = 10d / 360d;

    public static final double kAmpPivotP = 15;
    public static final double kAmpPivotI = 30;
    public static final double kAmpPivotD = 0.15;

    public static final double kAmpPivotPIDTolerance =  5d / 360d;
    
    // Motor IDs
    public static final int kIntakeMotorID = 10;
    public static final int kPivotMotorID = 11;

    public static final boolean kIntakeMotorInverted = false;
    public static final boolean kPivotMotorInverted = true;

    public static final int kPivotMotorSmartCurrentLimit = 50;

    public static final int kPivotEncoderID = 0;

    public static final double kPivotStowRotRaw = 0.69; // rotations
    
    // This is any point that the robot can't reach.

    public static final double kPivotAngleGround = 210;
    public static final double kPivotAngleSource = 100;
    public static final double kPivotAngleAmp = 80;
    public static final double kPivotAngleStow = 0;

    // negative is eject
    public static final double kIntakeSpeed = 0.5;
    public static final double kEjectSpeed = -0.5;
    public static final double kFeedSpeakerShooterSpeed = -1;
    public static final double kFeedAmpShooterSpeed = -1;
    public static final double kAmpSpeed = -0.5;
    public static final double kIntakeHoldSpeed = 0.2;

    public static final int kIntakeDefaultAmps = 40;
    public static final int kIntakeHoldAmps = 1;

    public static final int kTimeOfFlightSensorID = 40;

    public static final double kNoteIntakedSensorValue = 150;
    
  }

  public static final class ClimberConstants {
    // PIDs 
    public static final double kClimberP = 0.5;
    public static final double kClimberI = 0.1;
    public static final double kClimberD = 0.05;

    // Motor IDs
    public static final int kLeftClimberMotorID = 30;
    public static final int kRightClimberMotorID = 31;

    // Inverted
    public static final boolean kLeftClimberMotorInverted = false;
    public static final boolean kRightClimberMotorInverted = true;
    
    // Output Min/Max 
    public static final double kClimberMinOutput = 0;
    public static final double kClimberMaxOutput = 1;

    // Gear Ratio
    // public static final double kClimberGearRatio = 1d / 12d;

    public static final double kRotationsToUpLeft = 66;
    public static final double kRotationsToUpRight = 66;

    public static final double kTiltRPM = 20;
    
  }

  public static final class LEDConstants { 
    public static final int CANdleID = 50;
  }

  public static final class VisionConstants {
    public static final AprilTagFieldLayout LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

    public static final String FL_CAM_NAME = "str_fl_cam";
    public static final String FR_CAM_NAME = "str_fr_cam";
    public static final String BL_CAM_NAME = "str_bl_cam";
    //public static final String BR_CAM_NAME = "str_br_cam";

    public static final Transform3d flRobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(12.275412), Units.inchesToMeters(12.265), Units.inchesToMeters(8.430661)), new Rotation3d(0, Math.toRadians(-28.125), Math.toRadians(30)));
    public static final Transform3d frRobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(12.275412), Units.inchesToMeters(-12.265), Units.inchesToMeters(8.430661)), new Rotation3d(0, Math.toRadians(-28.125), Math.toRadians(-30)));
    public static final Transform3d blRobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-12.275412), Units.inchesToMeters(12.265), Units.inchesToMeters(8.430661)), new Rotation3d(0, Math.toRadians(-28.125), Math.toRadians(150)));
    //public static final Transform3d brRobotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-12.275412), Units.inchesToMeters(-12.265), Units.inchesToMeters(8.430661)), new Rotation3d(0, Math.toRadians(-28.125), Math.toRadians(-150)));

  }
}
