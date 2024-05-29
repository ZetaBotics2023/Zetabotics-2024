package frc.robot.DeprecatedSystems;

import javax.tools.Diagnostic;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutonConfigurationConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class VisionUpdaterThread extends Thread {
    PhotonPoseEstimator leftEstimator;
    PhotonPoseEstimator rightEstimator;

    SwerveDrivePoseEstimator poseEstimator;
    DriveSubsystem m_driveSubsystem;
    int numberOfVisionUpdates = 0;

    public VisionUpdaterThread(PhotonPoseEstimator leftEstimator, PhotonPoseEstimator rightEstimator,
        SwerveDrivePoseEstimator poseEstimator, DriveSubsystem m_driveSubsystem) {
        this.leftEstimator = leftEstimator;
        this.rightEstimator = rightEstimator;
        this.poseEstimator = poseEstimator;
        this.m_driveSubsystem = m_driveSubsystem;
    }

    @Override
    public void run() {
        SmartDashboard.putNumber("Updated Vision", numberOfVisionUpdates++);
        while(true) {
        if(!AutonConfigurationConstants.kInAuto || this.poseEstimator.getEstimatedPosition().getX() < 5) {
            try {
                leftEstimator.update().ifPresent(robotPose -> {
                    if (robotPose != null) {
                        // New pose from vision
                        Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();
                        poseEstimator.addVisionMeasurement(
                        new Pose2d(robotPose2d.getX() - .1, robotPose2d.getY() - .1, this.m_driveSubsystem.getHeadingInRotation2d()),
                        robotPose.timestampSeconds,
                         VisionConstants.kVisionMeasurementStdDevs);
                    }});
                rightEstimator.update().ifPresent(robotPose -> {
                    if (robotPose != null) {
                        // New pose from vision
                        Pose2d robotPose2d = robotPose.estimatedPose.toPose2d();
                        poseEstimator.addVisionMeasurement(
                        new Pose2d(robotPose2d.getX() - .1, robotPose2d.getY() - .1, this.m_driveSubsystem.getHeadingInRotation2d()),
                        robotPose.timestampSeconds,
                        VisionConstants.kVisionMeasurementStdDevs);
                    }});
                } catch (Exception e) {
                    SmartDashboard.putBoolean("Failed Vision", true);
                }
            }
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
                }
            }
    }
}
