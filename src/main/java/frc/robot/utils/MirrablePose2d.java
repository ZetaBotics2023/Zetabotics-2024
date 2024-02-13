package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

public class MirrablePose2d extends Pose2d{
    public MirrablePose2d(Pose2d pose2d, boolean shouldMirror) {
        super(shouldMirror ? FieldConstants.kWidth - pose2d.getX() : pose2d.getX(), pose2d.getY(), shouldMirror ? pose2d.getRotation().times(-1) : pose2d.getRotation());
    }
}
