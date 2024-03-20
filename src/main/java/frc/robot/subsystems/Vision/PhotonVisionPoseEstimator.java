package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.lang.reflect.AnnotatedType;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.InTeleop;
import frc.robot.Constants.AutonConfigurationConstants;
import frc.robot.Constants.FieldConstants;

public class PhotonVisionPoseEstimator extends SubsystemBase {
    private final PhotonCamera leftCamera = new PhotonCamera("LeftCamera");
    private final PhotonCamera centerCamera = new PhotonCamera("CenterCamera");
    private final PhotonCamera rightCamera = new PhotonCamera("RightCamera");

    private SwerveDrivePoseEstimator poseEstimator;
    private PhotonPoseEstimator leftEstimator;
    private PhotonPoseEstimator centerEstimator;
    private PhotonPoseEstimator rightEstimator;

    private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;

    private DriveSubsystem m_driveSubsystem;
    private final Field2d field2d = new Field2d();
    private final AprilTagFieldLayout aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public PhotonVisionPoseEstimator(DriveSubsystem m_driveSubsystem) {
        this.m_driveSubsystem = m_driveSubsystem;

        aprilTagLayout.setOrigin(this.originPosition);
        this.leftEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.AVERAGE_BEST_TARGETS, this.leftCamera,
                VisionConstants.kLeftCameraToRobot);
        this.centerEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.AVERAGE_BEST_TARGETS, this.centerCamera,
                VisionConstants.kCenterCameraToRobot);
        this.rightEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.AVERAGE_BEST_TARGETS, this.rightCamera,
                VisionConstants.kRightCameraToRobot);

        this.leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        this.centerEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        this.rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        poseEstimator = new SwerveDrivePoseEstimator(
                SwerveDriveConstants.kDriveKinematics,
                m_driveSubsystem.getHeadingInRotation2d(),
                m_driveSubsystem.getModulePositions(),
                new Pose2d(),
                VisionConstants.kStateStdDevs,
                VisionConstants.kVisionMeasurementStdDevs);

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    }

    public void periodic() {
        // Update pose estimator with drivetrain sensors
        poseEstimator.update(
            m_driveSubsystem.getHeadingInRotation2d(),
            m_driveSubsystem.getModulePositions());
        
        if(VisionConstants.useVision) {
            estimatorChecker(leftEstimator);
            estimatorChecker(rightEstimator);
            //estimatorChecker(centerEstimator);

        }
        Pose2d dashboardPose = getCurrentPose();
        if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
          // Flip the pose when red, since the dashboard field photo cannot be rotated
          dashboardPose = flipAlliance(dashboardPose);
        }
        field2d.setRobotPose(dashboardPose);    }

    private void estimatorChecker(PhotonPoseEstimator photonEstimator) {
        photonEstimator.update().ifPresent(robotPose -> {
            if (robotPose != null) {
                // New pose from vision
                Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();
                if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
                  robotPose2d = flipAlliance(robotPose2d);
                }
                poseEstimator.addVisionMeasurement(new Pose2d(robotPose2d.getTranslation(), this.m_driveSubsystem.getHeadingInRotation2d()), robotPose.timestampSeconds,
                    confidenceCalculator(robotPose));
            }
        });
    }

    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
        double smallestDistance = Double.POSITIVE_INFINITY;
        
        for (PhotonTrackedTarget target : estimation.targetsUsed) {
            Transform3d t3d = target.getBestCameraToTarget();
            double distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance) smallestDistance = distance;
        }

        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
            ? 1
            : Math.max(
                1,
                (estimation.targetsUsed.get(0).getPoseAmbiguity()
                    + Constants.VisionConstants.POSE_AMBIGUITY_SHIFTER)
                    * Constants.VisionConstants.POSE_AMBIGUITY_MULTIPLIER);
        double confidenceMultiplier = Math.max(
            1,
            (Math.max(
                1,
                Math.max(0, smallestDistance - Constants.VisionConstants.NOISY_DISTANCE_METERS)
                    * Constants.VisionConstants.DISTANCE_WEIGHT)
                * poseAmbiguityFactor)
                / (1
                    + ((estimation.targetsUsed.size() - 1) * Constants.VisionConstants.TAG_PRESENCE_WEIGHT)));

        return Constants.VisionConstants.kVisionMeasurementStdDevs.times(confidenceMultiplier);
    }

    public void setAlliance(Optional<Alliance> alliance) {
        Alliance currentAlliance = alliance.orElse(Alliance.Blue);
        boolean allianceChanged = false;

        switch (currentAlliance) {
            case Blue:
                aprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
                originPosition = OriginPosition.kBlueAllianceWallRightSide;
                AutonConfigurationConstants.kIsBlueAlliance = true;
                break;
            case Red:
                aprilTagLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
                originPosition = OriginPosition.kRedAllianceWallRightSide;
                AutonConfigurationConstants.kIsBlueAlliance = false;
                break;
            default:
                // No valid alliance data. Nothing we can do about it
        }
        if (allianceChanged) {
            // The alliance changed, which changes the coordinate system.
            // Since a tag may have been seen and the tags are all relative to the
            // coordinate system, the estimated pose
            // needs to be transformed to the new coordinate system.
            Pose2d newPose = flipAlliance(poseEstimator.getEstimatedPosition());
            newPose = new Pose2d(newPose.getX(), newPose.getY(), new Rotation2d());
            setCurrentPose(newPose);
        }

        Pose2d dashboardPose = getCurrentPose();
        if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
            // Flip the pose when red, since the dashboard field photo cannot be rotated
            dashboardPose = flipAlliance(dashboardPose);
        }
        field2d.setRobotPose(dashboardPose);
    }

    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(new Pose2d(
                new Translation2d(FieldConstants.kLength, FieldConstants.kWidth),
                new Rotation2d(Math.PI)));
    }

    private String getFomattedPose() {
        Pose2d pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f degrees",
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
            m_driveSubsystem.getHeadingInRotation2d(),
            m_driveSubsystem.getModulePositions(),
            newPose);
    }
    

    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    public void resetPose() {
        poseEstimator.resetPosition(
            m_driveSubsystem.getHeadingInRotation2d(),
            m_driveSubsystem.getModulePositions(),
            new Pose2d());
    }
    
    public void resetHolonomicRotation() {
        poseEstimator.resetPosition(
            Rotation2d.fromDegrees(0),
            m_driveSubsystem.getModulePositions(),
            getCurrentPose());
      }
}
