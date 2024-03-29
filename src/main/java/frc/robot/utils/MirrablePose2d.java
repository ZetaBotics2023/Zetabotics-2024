package frc.robot.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;

//TODO: Please for the love of all that is holy make this a method that returns a Pose2d
public class MirrablePose2d{
    private Pose2d pose2d;
    public MirrablePose2d(Pose2d pose2d) {
        this.pose2d = pose2d;
    }

    public Pose2d getPose(boolean shouldMirror) {
        if(shouldMirror) {
            return new Pose2d(this.pose2d.getX() + .1 , (FieldConstants.kWidth) - this.pose2d.getY() + .1, Rotation2d.fromDegrees(this.pose2d.getRotation().getDegrees() * -1));
        } //- .1, +.2
        return this.pose2d;

    }
}

