package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

/**
 * A utility class to allow us to calculate efficient robot shooting positions for the speaker
 */
public class CalculateSpeakerShootingPosition {

    /**
     * Gets the closest position and rotation
     * the robot can travel to in
     * order to shoot into the speaker
     * @param robotPositionMeters The Pose2d of the robot
     * @return The Pose2d representing the target position and rotation of the robot
     */
    public static Pose2d calculateTargetPosition(Pose2d robotPositionMeters) {
        // Run calculateTargetPosition with constants
        return calculateTargetPosition(robotPositionMeters, ShooterConstants.kMinShootingDistanceMeters, ShooterConstants.kMaxShootingDistanceMeters);
    }

    /**
     * This math will be used to determine the point 
     * that is a certain radius from the AprilTag and 
     * is closest to the robot, as well as the angle 
     * the robot needs in order to face the AprilTag.
     * @param robotPositionMeters The Pose2d of the robot
     * @param targetDistanceMinMeters The minimum distance the robot can shoot from
     * @param targetDistanceMaxMeters The maximum distance the robot can shoot from
     * @return A Pose2d containing the target position and rotation of the robot
     */
    public static Pose2d calculateTargetPosition(Pose2d robotPositionMeters, double targetDistanceMinMeters, double targetDistanceMaxMeters) {

        // Get which AprilTag we're centered around based on the alliance we're on
        Pose2d aprilTagPosition = (
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
            VisionConstants.kBlueAllianceShooterAprilTagPosition : VisionConstants.kRedAllianceShooterAprilTagPosition);

        // Calculate the distance between the robot and AprilTag in X and Y
        double distanceToAprilTagX = robotPositionMeters.getX() - aprilTagPosition.getX();
        double distanceToAprilTagY = robotPositionMeters.getY() - aprilTagPosition.getY();

        // Use Pythagorean theorem to calculate direct distance from robot to AprilTag
        double distanceToAprilTag = Math.sqrt(Math.pow(distanceToAprilTagX, 2) + Math.pow(distanceToAprilTagY, 2));

        // The conversion factor between the imaginary triangle of the robot and AprilTag, and the target point and the AprilTag
        double robotToTargetConversionFactorMin = targetDistanceMinMeters / distanceToAprilTag;
        double robotToTargetConversionFactorMax = targetDistanceMaxMeters / distanceToAprilTag;

        // The target position in the X axis
        double targetPositionXMin = (distanceToAprilTagX * robotToTargetConversionFactorMin) + aprilTagPosition.getX();
        double targetPositionXMax = (distanceToAprilTagX * robotToTargetConversionFactorMax) + aprilTagPosition.getX();

        // The target position in the Y axis
        double targetPositionYMin = (distanceToAprilTagY * robotToTargetConversionFactorMin) + aprilTagPosition.getY();
        double targetPositionYMax = (distanceToAprilTagY * robotToTargetConversionFactorMax) + aprilTagPosition.getY();

        // Calculate whether the minimum or maximum position is closest and therefore "more efficient" to travel to (saves about 0.4 meters)
        Translation2d targetTranslation = getClosestTargetPosition(
            robotPositionMeters, 
            new Translation2d(targetPositionXMin, targetPositionYMin), 
            new Translation2d(targetPositionXMax, targetPositionYMax),
            aprilTagPosition);


        // Calcuate the angle the robot will end with
        SmartDashboard.putNumber("Distence to April tag Y", distanceToAprilTagY);
        SmartDashboard.putNumber("Distence to April tag", distanceToAprilTag);
        double endPositionDistenceFromTagX = targetTranslation.getX() - aprilTagPosition.getX();
        double endPositionDistenceFromTagY = targetTranslation.getY() - aprilTagPosition.getY();
        double endPositionDistenceFromTag = Math.sqrt(Math.pow(endPositionDistenceFromTagX, 2) + Math.pow(endPositionDistenceFromTagY, 2));

        
        double targetAngleRadians = Math.asin(endPositionDistenceFromTagY/endPositionDistenceFromTag);
        
        if(robotPositionMeters.getY() > aprilTagPosition.getY()) {
            return new Pose2d(targetTranslation, Rotation2d.fromRadians(Math.abs(targetAngleRadians)));
        }
        // Construct the final position and return it
        return new Pose2d(targetTranslation, Rotation2d.fromRadians(targetAngleRadians));
    }

    /**
     * Returns the closest of two positions relative to the robot
     * @param robotPosition The Pose2d of the robot
     * @param positionOne The first translation to compare
     * @param positionTwo The second translation to compare
     * @return The closest
     */
    private static Translation2d getClosestTargetPosition(Pose2d robotPosition, Translation2d positionOne, Translation2d positionTwo, Pose2d aprilTagPosition) {
        // Calculate the difference between the robot and target positions
        double totalDifferenceOne = Math.abs(robotPosition.getX() - positionOne.getX()) + Math.abs(robotPosition.getY() - positionOne.getY());
        double totalDifferenceTwo = Math.abs(robotPosition.getX() - positionTwo.getX()) + Math.abs(robotPosition.getY() - positionTwo.getY());

        // Return the smallest one
        Translation2d finalPosition = totalDifferenceOne < totalDifferenceTwo ? positionOne : positionTwo;

        // If our end position relative to the AprilTag is too close to the wall, set it to a position along the radius that's far enough
        if (Math.abs(finalPosition.getX() - aprilTagPosition.getX()) < ShooterConstants.kMinShootingDistanceFromWallMeters) {
            
        }
    }
}
