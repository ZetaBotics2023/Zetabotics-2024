package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A helper class that represents an estimated position as returned from the Limelight
 */
public class VisionPose {
    private Pose2d pose;
    private double timeStamp;
    private boolean validTarget;

    public VisionPose(Pose2d pose, double timeStamp, boolean validTarget) {
        this.pose = pose;
        this.timeStamp = timeStamp;
        this.validTarget = validTarget;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getTimeStamp() {
        return this.timeStamp;
    }

    public boolean isValidTarget() {
        return this.validTarget;
    }
}
