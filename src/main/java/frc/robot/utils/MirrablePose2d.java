package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

//TODO: Please for the love of all that is holy make this a method that returns a Pose2d
public class MirrablePose2d extends Pose2d{
    public MirrablePose2d(Pose2d pose2d, boolean shouldMirror) {
        super(pose2d.getX(), (shouldMirror ? FieldConstants.kWidth - pose2d.getY() : pose2d.getY()),
         shouldMirror ? Rotation2d.fromDegrees(pose2d.getRotation().getDegrees() * -1) : pose2d.getRotation());
    }
}
