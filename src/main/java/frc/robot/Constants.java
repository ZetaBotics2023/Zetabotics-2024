// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SequentialGroupCommand;
import frc.robot.utils.MirrablePose2d;

/*
 * Contains constant values our code needs to use
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kButtonBoardPort = 1;
    public static final int kButtonBoardAltPort = 2;

    public static final double kDeadband = .01;
    public static String kLastAuto;
  }

  public static class SwerveDriveConstants {
    public static boolean driverController = true;
    public static final int kFrontLeftDriveMotorId = 24;
    public static final int kFrontRightDriveMotorId = 2;
    public static final int kBackLeftDriveMotorId = 3;
    public static final int kBackRightDriveMotorId = 4;

    public static final int kFrontLeftTurnMotorId = 5;
    public static final int kFrontRightTurnMotorId = 6;
    public static final int kBackLeftTurnMotorId = 7;
    public static final int kBackRightTurnMotorId = 8;

    public static final int kFrontLeftTurnEncoderId = 9;
    public static final int kFrontRightTurnEncoderId = 10;
    public static final int kBackLeftTurnEncoderId = 11;
    public static final int kBackRightTurnEncoderId = 12;

    // Make Sure to set these
    /*
     * public static final double kFrontLeftTurnEncoderOffset = 0.145508;
     * public static final double kFrontRightTurnEncoderOffset = 0.962402;
     * public static final double kBackLeftTurnEncoderOffset = 0.688477;
     * public static final double kBackRightTurnEncoderOffset = 0.837891;
     */

    public static final double kFrontLeftTurnEncoderOffset = 0.712646;// 0.144531;
    public static final double kFrontRightTurnEncoderOffset = 0.433838;// 0.965820;
    public static final double kBackLeftTurnEncoderOffset = 0.894043;// 0.934326;
    public static final double kBackRightTurnEncoderOffset = 0.440674;// 0.837646;

    public static final double kFrontLeftTurnMagnetOffset = 0.629395;
    public static final double kFrontRightTurnMagnetOffset = 0.139160;
    public static final double kBackLeftTurnMagnetOffset = 0.326660;
    public static final double kBackRightTurnMagnetOffset = 0.567627;

    public static final boolean kFrontLeftDriveMotorRev = false;
    public static final boolean kFrontRightDriveMotorRev = false;
    public static final boolean kBackLeftDriveMotorRev = false;
    public static final boolean kBackRightDriveMotorRev = false;

    public static final boolean kFrontLeftTurnMotorRev = true;
    public static final boolean kFrontRightTurnMotorRev = true;
    public static final boolean kBackLeftTurnMotorRev = true;
    public static final boolean kBackRightTurnMotorRev = true;

    public static final int kGyroId = 13;
    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 4.3;//3.9;// 4.6;
    public static final double kMaxRotationAnglePerSecond = 11.4;// 12;

    public static final double kRadiusFromCenterToSwerves = 1.0;

    // Last years values
    public static final double kDistanceBetweenCentersOfRightAndLeftWheels = .60325;// 0.6096;
    public static final double kDistanceBetweenCentersOfFrontAndBackWheels = .60325;// 0.6096;
    public static final double kRadiusFromCenterToFarthestSwerveModule = Math
        .sqrt(((kDistanceBetweenCentersOfRightAndLeftWheels * kDistanceBetweenCentersOfRightAndLeftWheels)
            + (kDistanceBetweenCentersOfFrontAndBackWheels * kDistanceBetweenCentersOfFrontAndBackWheels)));

    // These are 100% Good
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            -kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            -kDistanceBetweenCentersOfRightAndLeftWheels / 2));

    public static final double kPModuleTurningController = 3;// .000000000000000001;//.00000001;
    public static final double kIModuleTurningController = .0;
    public static final double kDModuleTurningController = .0;

    public static final double kPositionToleranceDegrees = .1;
    public static final double kVelocityToleranceDegreesPerSec = 1.0;

    public static final double kMaxModuleAngularSpeedDegreesPerSecond = 30.0;
    public static final double kMaxModuleAngularAccelDegreesPerSecondSquared = 30.0;

    public static final double kTranslationRateLimiter = 8;// 9;
    public static final double kRotationRateLimiter = 100;// 20;
  }

  public static class SwerveModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * 2 * Math.PI;

    // Set to the last years values
    public static final double kPModuleDriveController = 0.0001;
    public static final double kIModuleDriveController = .00000125;// .0000025;//.000002;
    public static final double kDModuleDriveController = 0;// 0.0001;
    public static final double kFModuleDriveController = 0;
    public static final double kIZoneModuleDriveController = 0.0;

    public static final double kPModuleTurningController = .1;
    public static final double kIModuleTurningController = 0;
    public static final double kDModuleTurningController = 0;
    public static final double kFModuleTurningController = 0;
    public static final double kIZoneModuleTurningController = 0.5 / 360; // .5 degrees converted to rotations

    // Updated for this year
    public static final double kAbsoluteTurningEncoderCPR = 4096.0;
    public static final double kNeoEncoderCPR = 4096.0;
    public static final double kMaxRPM = 5676.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.80945463);// 3.83931974);//0.1016;
    public static final double kDriveGearRatio = (50.0 * 17.0 * 45.0) / (14.0 * 27.0 * 15.0);// 6.75/1.0;
    public static final double kTurningGearRatio = 150.0 / 7.0;

    public static final double kTurningConversionFactor = 360.0 / kTurningGearRatio;

    public static final double kAbsoluteTurningEncoderCPRToDegrees = (kAbsoluteTurningEncoderCPR
        / kAbsoluteTurningEncoderCPR) * 360.0;

    public static final double kAbsoluteTurningEncoderCPRToDegreesMult = 360.0 / 4096.0;

    public static final double kRelativeTurningEncoderDegreesToCPRMult = kNeoEncoderCPR / 360;
    // ((kNeoEncoderCPR / kNeoEncoderCPR) * 360) * kTurningGearRatio;

    public static final double kWheelDistancePerRotation = kWheelDiameterMeters * Math.PI;

    public static final double kDriveConversionPositionFactor = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;
    public static final double kDriveConversionVelocityFactor = kDriveConversionPositionFactor / 60.0;

    public static final double kDriveEncoderDistancePerPulse = ((kWheelDiameterMeters * Math.PI) / kDriveGearRatio)
        / kNeoEncoderCPR;

    public static final double kTurningEncoderRadiansPerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / kAbsoluteTurningEncoderCPR;
  }

  public static final class FieldConstants {
    // Current length is the one pathplanner uses the one I caclulated is slightly
    // larger at 16.59127999998984m if the red aleicne paths are undershootingss in
    // the X axis maybe change it to the one I calculated
    public static final double kLength = 16.541749;// 16.45;//Units.feetToMeters(54.2708);//Units.feetToMeters(54);
    public static final double kWidth = Units.feetToMeters(26.9375);
    public static Optional<Alliance> alliance;
  }

  public static final class VisionConstants {
    // Limelight pose
    // 13in horizontal
    // 16 1/4in vertical
    // 21 Degrees Pitch
    public static final Pose2d kRedAllianceShooterAprilTagPosition = new Pose2d(new Translation2d(16.579342, 5.547868),
        Rotation2d.fromDegrees(180));
    public static final Pose2d kBlueAllianceShooterAprilTagPosition = new Pose2d(new Translation2d(-0.0381, 5.547868),
        Rotation2d.fromDegrees(0));
    public static final double kDegreeOffset = 3.58;
// 20 yaw, 20 pitch, -1cm from ll, 1.5cm lower, cm apart
// 7.2cm away from ll on left
// 
    public static final Transform3d kLeftCameraToRobot = new Transform3d(-0.342425, .072, .49935, new Rotation3d(0, Math.toRadians(-20), (Math.toRadians(180))));
    public static final Transform3d kCenterCameraToRobot = new Transform3d(-0.352425, 0, .51435, new Rotation3d(0, Math.toRadians(-22.5), (Math.toRadians(180))));
    public static final Transform3d kRightCameraToRobot = new Transform3d(-0.342425, -.072, .49935, new Rotation3d(0, Math.toRadians(-20), (Math.toRadians(160))));


        public static final Transform3d robotToCam = new Transform3d(-0.352425, 0, .51435 - 1.5, new Rotation3d(0, Math.toRadians(20), (Math.toRadians(200))));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;
    public static final Vector<N3> kStateStdDevs = VecBuilder.fill(0.1, 1, 0.1);
    public static final Vector<N3> kVisionMeasurementStdDevs = VecBuilder.fill(.4, .4, 0);//(1, 1, 1);
    public static boolean useVision = true;
  };

  public static final class AutoConstants {
    public static final PIDConstants kTranslationAutoPID = new PIDConstants(1.95, 0, .0001);
    public static final PIDConstants kRotationAutoPID = new PIDConstants(20, 0, 0);//new PIDConstants(3.5, 0.0, 0.0);
    public static final double kMaxAutonSpeedInMetersPerSecond = 2.1;
    public static final double kMaxAutonAccelerationInMetersPerSecondSqr = 3;

    public static final double kAutoSlowDownSpeed = .75;
    public static final double kAutoSlowRate = 1.6;
    public static final double kAutoSlowRateAuto = 1.9;

    // Auto Constrants

    public static final double kMaxTranslationSpeedMPSAuto = 4.3;
    public static final double kMaxTranslationAccelerationAuto = 10;//4.1;// 2;//4.1;
    public static final TrapezoidProfile.Constraints kTranslationAutoControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxTranslationSpeedMPSAuto, kMaxTranslationAccelerationAuto);

    public static final double kTranslationAutoPIDControllerVelocityTolerance = 1;
    public static final double kTranslationAutoPIDControllerPositionalTolerance = .05;// .05;

    public static final double kTranslationAutoPIDControllerP = .9265;//1.8;
    public static final double kTranslationAutoPIDControllerI = 0;
    public static final double kTranslationAutoPIDControllerD = 0;

    // Heading Auto
    public static final double kHeadingPIDControllerAutoP = .0585;//.06;//.035;
    public static final double kHeadingPIDControllerAutoI = 0.0;//.00005;
    public static final double kHeadingPIDControllerAutoD = 0;
    public static final double kHeadingPIDControllerToleranceAuto = 2;
    public static final double kMaxAngularSpeedRadiansPerSecondAuto = 180;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquaredAuto = 720;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraintsAuto = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecondAuto, kMaxAngularAccelerationRadiansPerSecondSquaredAuto);
    public static final double kAutoPositonToleranceAuto = .075;
    public static final double kAutoPositonToleranceAutoHigh = .15;

    // To To Pose Teleop
    public static final double kFirstBatteryPIDLimit = 11;
    public static final double kSecondBatteryPIDLimit = 10;
    public static final double kThirdBatteryPIDLimit = 9;

    public static final double kMaxTranslationSpeedMPS = 4.3;
    public static final double kMaxTranslationAcceleration = 10;
    public static final TrapezoidProfile.Constraints kTranslationControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxTranslationSpeedMPS, kMaxTranslationAcceleration);

    public static final double kMaxTranslationSpeedMPSLowVoltage = 2;
    public static final double kMaxTranslationAccelerationLowVoltage = 2;
    public static final TrapezoidProfile.Constraints kTranslationControllerConstraintsLowVoltage = new TrapezoidProfile.Constraints(
        kMaxTranslationSpeedMPSLowVoltage, kMaxTranslationAccelerationLowVoltage);

    public static final double kTranslationPIDControllerP = .9265;//1.8// 1.4;//1.5;
    public static final double kTranslationPIDControllerI = 0;
    public static final double kTranslationPIDControllerD = 0;

    public static final double kTranslationPIDControllerPFirstBatteryPIDLimit = 1.5;
    public static final double kTranslationPIDControllerIFirstBatteryPIDLimit = 0;
    public static final double kTranslationPIDControllerDFirstBatteryPIDLimit = 0;

    public static final double kTranslationPIDControllerPSecondBatteryPIDLimit = 1;
    public static final double kTranslationPIDControllerISecondBatteryPIDLimit = 0;
    public static final double kTranslationPIDControllerDSecondBatteryPIDLimit = 0;

    public static final double kTranslationPIDControllerPThirdBatteryPIDLimit = .75;
    public static final double kTranslationPIDControllerIThirdBatteryPIDLimit = 0;
    public static final double kTranslationPIDControllerDThirdBatteryPIDLimit = 0;

    public static final double kTranslationPIDControllerVelocityToleranceHigh = 2;
    public static final double kTranslationPIDControllerVelocityTolerance = .25;
    public static final double kTranslationPIDControllerPositionalToleranceHigh = .2;
    public static final double kTranslationPIDControllerPositionalTolerance = .05;

    public static final double kHeadingPIDControllerP = .0585;//.035;
    public static final double kHeadingPIDControllerI = 0;//.00005;
    public static final double kHeadingPIDControllerD = 0;

    public static final double kHeadingPIDControllerPFirstBatteryPIDLimit = .035;
    public static final double kHeadingPIDControllerIFirstBatteryPIDLimit = .00005;
    public static final double kHeadingPIDControllerDFirstBatteryPIDLimit = 0;

    public static final double kHeadingPIDControllerPSecondBatteryPIDLimit = .035;
    public static final double kHeadingPIDControllerISecondBatteryPIDLimit = .00005;
    public static final double kHeadingPIDControllerDSecondBatteryPIDLimit = 0;

    public static final double kHeadingPIDControllerPThirdBatteryPIDLimit = .035;
    public static final double kHeadingPIDControllerIThirdBatteryPIDLimit = .00005;
    public static final double kHeadingPIDControllerDThirdBatteryPIDLimit = 0;

    public static final double kHeadingPIDControllerTolerance = 1.2;
    public static final double kHeadingPIDControllerToleranceHigh = 4;

    public static final double kMaxAngularSpeedRadiansPerSecond = 360;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 720;

    public static final double kMaxAngularSpeedRadiansPerSecondLowVoltage = 270;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquaredLowVoltage = 360;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraintsLowVoltage = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecondLowVoltage, kMaxAngularAccelerationRadiansPerSecondSquaredLowVoltage);

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final double kAutoPositonTolerance = kTranslationPIDControllerPositionalTolerance;
  }

  public static final class WPILIBTrajectoryConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.1;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI * 4;

    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared)
                      .setKinematics(SwerveDriveConstants.kDriveKinematics);

    public static final PIDController kXController = new PIDController(1.95, 0, .0001);
    public static final PIDController kYController = new PIDController(1.95, 0, .0001);
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final ProfiledPIDController kThetaController = new ProfiledPIDController(20, 0, 0, kThetaControllerConstraints);
    
  }

  public static final class AutonConfigurationConstants {
    public static final ArrayList<String> kConfiguredAutonNames = new ArrayList<String>();
    public static final HashMap<String, SequentialGroupCommand> kConfiguredAutons = new HashMap<String, SequentialGroupCommand>();

    public static final HashMap<String, MirrablePose2d> robotPositions = new HashMap<String, MirrablePose2d>();

    public static boolean kIsBlueAlliance = false;// DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

    public static final ArrayList<Command> kLeft_ShootPreloaded = new ArrayList<Command>();
    public static final ArrayList<Command> kLeft_ShootPreloadedLeft = new ArrayList<Command>();
    public static final ArrayList<Command> kLeft_ShootPreloadedLeftCenter = new ArrayList<Command>();
    public static final ArrayList<Command> kLeft_ShootPreloadedLeftCenterRight = new ArrayList<Command>();

    public static final ArrayList<Command> kCenter_ShootPreloaded = new ArrayList<Command>();;
    public static final ArrayList<Command> kCenter_ShootPreloadedCenter = new ArrayList<Command>();
    public static final ArrayList<Command> kCenter_ShootPreloadedCenterLeft = new ArrayList<Command>();
    public static final ArrayList<Command> kCenter_ShootPreloadedCenterRight = new ArrayList<Command>();

    public static final ArrayList<Command> kRight_ShootPreloaded = new ArrayList<Command>();;
    public static final ArrayList<Command> kRight_ShootPreloadedRight = new ArrayList<Command>();
    public static final ArrayList<Command> kRight_ShootPreloadedRightCenter = new ArrayList<Command>();

    public static final ArrayList<Command> kLeft_ShootPreloadedFarFarLeft = new ArrayList<Command>();
    public static final ArrayList<Command> kLeft_ShootPreloadedFarFarLeftFarLeft = new ArrayList<Command>();

    public static final MirrablePose2d kLeftStartingPose = new MirrablePose2d(
        new Pose2d(1.5134 - Units.inchesToMeters(3), 7, new Rotation2d()));
    public static final MirrablePose2d kCenterStartingPose = new MirrablePose2d(
        new Pose2d(1.5134 - Units.inchesToMeters(3), 5.55, new Rotation2d()));
    public static final MirrablePose2d kRightStartingPose = new MirrablePose2d(
        new Pose2d(1.5134 - Units.inchesToMeters(3), 4.11, new Rotation2d()));

    public static final MirrablePose2d kLeftFarNoteStartingPose = new MirrablePose2d(
        new Pose2d(1.5134, 6.2, new Rotation2d(0)));

    public static final double kLeftNoteIntakeDownTime = .4;//1;// .9;//.75;
    public static final double kCenterNoteIntakeDownTime = 1.15;
    public static final double kRightNoteIntakeDownTime = 1;
    public static boolean kSTOP = false;
  }

  public static final class ShooterConstants {
    public static boolean kRampShooter = false;

    public static final int kLeftShooterMotorControllerID = 16;
    public static final int kRightShooterMotorControllerID = 17;

    public static final double kLeftShooterGearRatio = 1.0;
    public static final double kRightShooterGearRatio = 1.0;

    public static final double kPShooterController = 0.00025;
    public static final double kIShooterController = 9.999999974752427e-7;// 0.0000001;;
    public static final double kDShooterController = 0.0;
    public static final double kFLeftShooterController = 0.0;
    public static final double kIZoneShooterController = 0.0;

    public static final double kMinShootingDistanceMeters = 2.1;
    public static final double kMaxShootingDistanceMeters = 2.1;

    public static final double kMinShootingDistanceFromWallMeters = Units.inchesToMeters(40);

    public static final double kShooterPowerRatio = 1;
    public static double kShooterRPM = 2800;//2800;// SmartDashboard.getNumber("Shooter RPM", 4200);//4200;//4500;
    public static final double kShootTime = 10;
    public static final double kShootTimeAuto = .4;
    public static final double kShooterRPMTolerance = 100;

    public static double kShooterRPMChange = 0;

    public static final MirrablePose2d kLeftShootingPose = new MirrablePose2d(
        new Pose2d(2.1, 6.6, Rotation2d.fromDegrees(24)));
    public static final MirrablePose2d kCenterShootingPose = new MirrablePose2d(
        new Pose2d(2.3, 5.55, new Rotation2d()));// 2.3
    public static final MirrablePose2d kRightShootingPose = new MirrablePose2d(
        new Pose2d(2.1, 4.6, Rotation2d.fromDegrees(-26)));
  }

  public static final class ClimberConstants {
    public static final int kLeftClimberMotorControllerID = 23;
    public static final int kRightClimberMotorControllerID = 22;

    public static final float kClimberMaxHeight = 345;
    public static final float kClimberMinHeight = 3.5f;

    public static final double kClimbUpPercentOutput = 1;
    public static final double kClimbDownPercentOutput = -1;
  }

  public static final class IntakeConstants {
    public static final int kPivotMotorControllerID = 14;
    public static final int kLeftPivotID = 25;
    public static final int kRightPivotID = 26;

    public static final int kIntakeMotorControllerID = 15;
    public static final int distenceSensorID = 0;

    public static final double kPivotThroughBoreZeroOffset = 0.105469 * 360;//93.75;

    public static final double kFarthestNotePositionMillimeters = 480;//480;

    public static final double kPivotGearRatio = 60.0 / 1.0;
    public static final double kIntakeGearRatio = 4.0 / 1.0;

    public static final double kPPivotController = 3;
    public static final double kIPivotController = 0;
    public static final double kDPivotController = 0;
    public static final double kFPivotController = 0;
    public static final double kIZonePivotController = 0.5 / 360.0; // .5 degrees in rotations

    public static final double kPPivotControllerProfiled = 3;
    public static final double kIPivotControllerProfiled = 0;
    public static final double kDPivotControllerProfiled = 0;
    public static final double kPivotAngleTolorence = .5;
    public static final double maxPivotVelocity = 400;
    public static final TrapezoidProfile.Constraints kPivotContraints = new TrapezoidProfile.Constraints(
      maxPivotVelocity, 1000);

    public static final double kPIntakeVelocityController = 0.000015;
    public static final double kIIntakeVelocityController = 0.0000001;
    public static final double kDIntakeVelocityController = 0.0;
    public static final double kFIntakeVelocityController = 0.0;
    public static final double kIZoneIntakeVelocityController = 0.0;

    public static final double kPIntakePositionController = 1;
    public static final double kIIntakePositionController = 0.0;
    public static final double kDIntakePositionController = 0.0;
    public static final double kFIntakePositionController = 0.0;
    public static final double kIZoneIntakePositionController = .5 / 360;

    // Rotation constants
    public static final double kGroundPickupPivotRotationDegrees = 194.16;//190.5;// 189;
    public static final double kGroundPickupIntakeRPM = 3000;//5200;

    public static final double kHumanPlayerPickupPivotRotationDegrees = 0.0;
    public static final double kHumanPlayerPickupIntakeRPM = 0.0;

    public static final double kShootInAmpPivotRotationDegrees = 120;
    public static final double kShootInAmpIntakeRPM = -1000;


    public static final double kPassIntoShooterPivotRotationDegrees = 0;
    public static final double kPassIntoShooterIntakeRPM = -3000;

    public static final double kPivotRotationToleranceDegrees = 5;
    public static final double kGroundPickupMinimumPosition = 100;
    public static final double kShootInAmpIntakeTime = 2;
    public static final int kPivotEncoderID = 41;
  }

  public static final class LEDConstants {
    public static final int kLEDPWMPort = 9;
    public static final int kLEDLength = 150;
    public static final int kLEDCANID = 30;
    public static int[] kDefultColor = new int[] {0, 255, 255};
    public static int[] kIntakeDownColor = new int[] {255, 0, 0};


  }
}
