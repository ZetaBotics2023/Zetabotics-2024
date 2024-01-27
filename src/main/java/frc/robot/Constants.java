// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = .1;
  }

  public static class SwerveDriveConstants {
    
    public static final int kFrontLeftDriveMotorId = 1;
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
    /*public static final double kFrontLeftTurnEncoderOffset = 0.145508;
    public static final double kFrontRightTurnEncoderOffset = 0.962402;
    public static final double kBackLeftTurnEncoderOffset = 0.688477;
    public static final double kBackRightTurnEncoderOffset = 0.837891;*/

    public static final double kFrontLeftTurnEncoderOffset = 0.144531;//0.146240;
    public static final double kFrontRightTurnEncoderOffset = 0.965820;//0.964111;
    public static final double kBackLeftTurnEncoderOffset = 0.688965;//0.685547;
    public static final double kBackRightTurnEncoderOffset = 0.837646;//0.837891;

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
    public static final boolean kBackRightTurnMotorRev =  true;

    public static final int kGyroId = 13;
    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 4.6;
    public static final double kMaxRotationAnglePerSecond = 6;

    public static final double kRadiusFromCenterToSwerves = 1.0;

    // Last years values
    
    public static final double kDistanceBetweenCentersOfRightAndLeftWheels = .60325;//0.6096;
    public static final double kDistanceBetweenCentersOfFrontAndBackWheels = .60325;//0.6096;
    public static final double kRadiusFromCenterToFarthestSwerveModule = Math.sqrt(((kDistanceBetweenCentersOfRightAndLeftWheels * kDistanceBetweenCentersOfRightAndLeftWheels) 
    + (kDistanceBetweenCentersOfFrontAndBackWheels * kDistanceBetweenCentersOfFrontAndBackWheels)));


    // These are 100% Good
    public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2, kDistanceBetweenCentersOfRightAndLeftWheels / 2),
                new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2, -kDistanceBetweenCentersOfRightAndLeftWheels / 2),
                new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2, kDistanceBetweenCentersOfRightAndLeftWheels / 2),
                new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2, -kDistanceBetweenCentersOfRightAndLeftWheels / 2));

    public static final double kPModuleTurningController = 3;//.000000000000000001;//.00000001;
    public static final double kIModuleTurningController = .0;
    public static final double kDModuleTurningController = .0;

    public static final double kPostitionToleranceDegrees = .1;
    public static final double kVelocityToleranceDegreesPerSec = 1.0;

    public static final double kMaxModuleAngularSpeedDegreesPerSecond =  30.0;
    public static final double kMaxModuleAngularAccelDegreesPerSecondSquared = 30.0;

    public static final double kTranslationRateLimiter = 6;//9;
    public static final double kRotationRateLimiter = 20;
  }

  public static class SwerveModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * 2 * Math.PI;

    // Set to the last years values
    public static final double kPModuleDriveController = 0.0001;
    public static final double kIModuleDriveController =  .00000125;//.0000025;//.000002;
    public static final double kDModuleDriveController = 0.0001;
    public static final double kFModuleDriveController = 0;
    public static final double kIZoneModuleDriveController = 0.0;

    public static final double kPModuleTurningController = .1;
    public static final double kIModuleTurningController = 0;
    public static final double kDModuleTurningController = 0;
    public static final double kFModuleTurningController = 0;
    public static final double kIZoneModuleTurningController = 0.5/360; // .5 degrees converted to rotations
  
    // Updated for this year
    public static final double kAbsoluteTurningEncoderCPR = 4096.0;
    public static final double kNeoEncoderCPR = 4096.0;
    public static final double kMaxRPM = 5676.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.685);//0.1016;
    public static final double kDriveGearRatio = (50.0 * 17.0 * 45.0) / (14.0 * 27.0 * 15.0);//6.75/1.0;
    public static final double kTurningGearRatio = 150.0/7.0; 

    public static final double kTurningConversionFactor = 360.0 / kTurningGearRatio;

    public static final double kAbsoluteTurningEncoderCPRToDegrees = 
    (kAbsoluteTurningEncoderCPR / kAbsoluteTurningEncoderCPR) * 360.0;

    public static final double kAbsoluteTurningEncoderCPRToDegreesMult = 360.0 / 4096.0;

    public static final double kRelativeTurningEncoderDegreesToCPRMult = kNeoEncoderCPR / 360;
    //((kNeoEncoderCPR / kNeoEncoderCPR) * 360) * kTurningGearRatio;

    public static final double kWheelDistancePerRotation = kWheelDiameterMeters * Math.PI;

    public static final double kDriveConversionPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;
    public static final double kDriveConversionVelocityFactor = kDriveConversionPositionFactor / 60.0;


    public static final double kDriveEncoderDistancePerPulse =
        ((kWheelDiameterMeters * Math.PI) / kDriveGearRatio) / kNeoEncoderCPR;

    public static final double kTurningEncoderRadiansPerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / kAbsoluteTurningEncoderCPR;
  }

  public static final class FieldConstants {
    public static final double kLength = Units.feetToMeters(54);
    public static final double kWidth = Units.feetToMeters(27);
  }


  public static final class VisionConstants {

    public static final Pose2d kRedAllianceShooterAprilTagPosition = new Pose2d(new Translation2d(16.579342, 5.547868), Rotation2d.fromDegrees(180));
    public static final Pose2d kBlueAllianceShooterAprilTagPosition = new Pose2d(new Translation2d(-0.0381, 5.547868), Rotation2d.fromDegrees(0));

  };

  public static final class AutoConstants {
    public static HashMap<String, Command> namedEventMap = new HashMap<>();
    public static Alliance alliance;
    public static final double kMaxAutonSpeedInMetersPerSecond = 4.1;
    public static final double kMaxAutonAccelerationInMetersPerSecondSqr = 4.1;

    public static final double kMaxAutonAngulerSpeedInMetersPerSecond = 4.1;
    public static final double kMaxAutonAngulerAccelerationInMetersPerSecondSqr = 4.1;
    //5.9
    public static final PIDConstants kTranslationAutoPID = new PIDConstants(2.5, 0, .0001);
    public static final PIDConstants kRotationAutoPID = new PIDConstants(4, 0.0, 0.0);
  }
  public static final class ShooterConstants {
    public static final int kLeftShooterMotorControllerID = 16;
    public static final int kRightShooterMotorControllerID = 17;

    public static final double kLeftShooterGearRatio = 1.0;
    public static final double kRightShooterGearRatio = 1.0;

    public static final double kPLeftShooterController = 0.0;
    public static final double kILeftShooterController = 0.0;
    public static final double kDLeftShooterController = 0.0;
    public static final double kFLeftShooterController = 0.0;
    public static final double kIZoneLeftShooterController = 0.0; 

    public static final double kPRightShooterController = 0.0;
    public static final double kIRightShooterController = 0.0;
    public static final double kDRightShooterController = 0.0;
    public static final double kFRightController = 0.0;
    public static final double kIZoneRightShooterController = 0.0; 


  }

  public static final class IntakeConstants {
    public static final int kPivotMotorControllerID = 14; //TODO: Change this to the real one
    public static final int kIntakeMotorControllerID = 15; //TODO: This one too
    public static final int distenceSensorID = 16;

    public static final double farthestNotePositionMilameters = Units.inchesToMeters(11) * 1000; 


    public static final double kPivotGearRatio = 100.0/1.0;
    public static final double kIntakeGearRatio = 3.0/1.0;

    

    //TODO: Tune me!
    public static final double kPPivotController = 0.0003;
    public static final double kIPivotController = .00000;
    public static final double kDPivotController = 0.0;
    public static final double kFPivotController = 0.0;
    public static final double kIZonePivotController = 0.5/360.0; // .5 degrees in rotations

    public static final double kPIntakeController = 0.0003;
    public static final double kIIntakeController = .00000;
    public static final double kDIntakeController = 0.0;
    public static final double kFIntakeController = 0.0;
    public static final double kIZoneIntakeController = 0.0;

    // Rotation constants
    public static final double kGroundPickupPivotRotationDegrees = 0.0;
    public static final double kGroundPickupIntakeRPM = 0.0;

    public static final double kHumanPlayerPickupPivotRotationDegrees = 0.0;
    public static final double kHumanPlayerPickupIntakeRPM = 0.0;

    public static final double kShootInAmpPivotRotationDegrees = 0.0;
    public static final double kShootInAmpIntakeRPM = 0.0;

    public static final double kPassIntoShooterPivotRotationDegrees = 0.0;
    public static final double kPassIntoShooterIntakeRPM = 0.0;

  }
}
