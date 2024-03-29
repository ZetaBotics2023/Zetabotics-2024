package frc.robot.DeprecatedSystems;

import java.io.IOException;
import java.lang.reflect.AnnotatedType;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

// We should probable swich this over to make use of WPILib SwerveDrive PoseEstimator and Limelight tag reading rather than photon vission
public class PoseEstimatorSubsystem extends SubsystemBase {

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);// gray matter has lower

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(.4, .4, 0); // gray mater has higher

  private final DriveSubsystem m_driveSubsystem;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();

  private final PhotonPoseEstimator leftEstimator;
  private final PhotonPoseEstimator rightEstimator;


  private double previousPipelineTimestamp = 0;
  private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;

  private final ArrayList<Double> xValues = new ArrayList<Double>();
  private final ArrayList<Double> yValues = new ArrayList<Double>();

  private PhotonCamera leftCamera = new PhotonCamera("LeftCamera");
  private PhotonCamera rightCamera = new PhotonCamera("RightCamera");

  public VisionUpdaterThread updater;


  public PoseEstimatorSubsystem(DriveSubsystem m_driveSubsystem) {
    this.m_driveSubsystem = m_driveSubsystem;
    
    // try {
    var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    layout.setOrigin(originPosition);
    // The Pose Strategy may be incorrect
    this.leftEstimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, leftCamera,
        Constants.VisionConstants.kLeftCameraToRobot);
    this.leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    this.rightEstimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, rightCamera,
        Constants.VisionConstants.kRightCameraToRobot);
    this.rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    
    // } catch (IOException e) {
    // DriverStation.reportError("Failed to load AprilTagFieldLayout",
    // e.getStackTrace());
    // photonPoseEstimator = null;
    // }

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator = new SwerveDrivePoseEstimator(
        SwerveDriveConstants.kDriveKinematics,
        m_driveSubsystem.getHeadingInRotation2d(),
        m_driveSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    
    this.updater = new VisionUpdaterThread(leftEstimator, rightEstimator, poseEstimator, m_driveSubsystem);
    this.updater.start();
  }

  /**
   * Sets the alliance. This is used to configure the origin of the AprilTag map
   * 
   * @param currentAlliance alliance
   */
  public void setAlliance(Optional<Alliance> alliance) {
    var fieldTags = leftEstimator.getFieldTags();

    Alliance currentAlliance = alliance.orElse(Alliance.Blue);
    boolean allianceChanged = false;
    switch (currentAlliance) {
      case Blue:
        fieldTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
        originPosition = OriginPosition.kBlueAllianceWallRightSide;
        AutonConfigurationConstants.kIsBlueAlliance = true;
        break;
      case Red:
        fieldTags.setOrigin(OriginPosition.kRedAllianceWallRightSide);
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
  }

  @Override
  public void periodic() {
    // Update pose estimator with drivetrain sensors
    try {
      poseEstimator.update(
        m_driveSubsystem.getHeadingInRotation2d(),
        m_driveSubsystem.getModulePositions());    
    } catch(Exception e) {
      SmartDashboard.putBoolean("Pose Estimater Crash", true);
    }
    
    
      
    
    Pose2d dashboardPose = getCurrentPose();
    if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = flipAlliance(dashboardPose);
    }
    field2d.setRobotPose(dashboardPose);
    SmartDashboard.putNumber("Robot X", this.poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Robot Y", this.poseEstimator.getEstimatedPosition().getY());
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        m_driveSubsystem.getHeadingInRotation2d(),
        m_driveSubsystem.getModulePositions(),
        newPose);
  }

  public void resetPose() {
    poseEstimator.resetPosition(
        m_driveSubsystem.getHeadingInRotation2d(),
        m_driveSubsystem.getModulePositions(),
        new Pose2d());
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  /**
   * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
   * always on the right corner of your
   * alliance wall, so for 2023, the field elements are at different coordinates
   * for each alliance.
   * 
   * @param poseToFlip pose to transform to the other alliance
   * @return pose relative to the other alliance's coordinate system
   */
  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(new Pose2d(
        new Translation2d(FieldConstants.kLength, FieldConstants.kWidth),
        new Rotation2d(Math.PI)));
  }

  /**
   * Resets the holonomic rotation of the robot (gyro last year)
   * This would be used if Apriltags are not getting accurate pose estimation
   */
  public void resetHolonomicRotation() {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(0),
        m_driveSubsystem.getModulePositions(),
        getCurrentPose());
  }

  public void resetPoseRating() {
    xValues.clear();
    yValues.clear();
  }
}