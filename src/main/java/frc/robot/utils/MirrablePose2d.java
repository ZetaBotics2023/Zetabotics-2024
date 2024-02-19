package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

//TODO: Please for the love of all that is holy make this a method that returns a Pose2d
public class MirrablePose2d extends Pose2d{
    public MirrablePose2d(Pose2d pose2d, boolean shouldMirror) {
        super(shouldMirror ? FieldConstants.kLength - pose2d.getX() : pose2d.getX(), pose2d.getY(),
         shouldMirror ? new Rotation2d(Math.PI).minus(pose2d.getRotation()) : pose2d.getRotation());
    }
}
