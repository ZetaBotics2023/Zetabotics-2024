package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A helper class that represents an estimated position as returned from the Limelight
 */
public class VisionPose {
    private Pose3d pose;
    private double timeStamp;
    private boolean validTarget;

    public VisionPose(Pose3d pose, double timeStamp, boolean validTarget) {
        this.pose = pose;
        this.timeStamp = timeStamp;
        this.validTarget = validTarget;
    }

    public Pose2d getPose() {
        return new Pose2d(new Translation2d(this.pose.getX(), this.pose.getY()), Rotation2d.fromRadians(this.pose.getRotation().getAngle()));
    }

    public double getTimeStamp() {
        return this.timeStamp;
    }

    public boolean isValidTarget() {
        return this.validTarget;
    }
}
