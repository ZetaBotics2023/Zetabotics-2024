package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

/**
 * A utility class to allow us to calculate efficient robot shooting positions for the speaker
 */
public class CalculateGoToPoseVelocityAuton {

    public static Translation2d calculateGoToPoseVelocity(Pose2d robotPose, Pose2d goalPose) {
        double robotDistenceToGoalX = Math.abs(goalPose.getX() - robotPose.getX());
        double robotDistenceToGoalY = Math.abs(goalPose.getY() - robotPose.getY());

        double robotVelocityX = 0;
        double robotVelocityY = 0;

        if(robotDistenceToGoalX < 1 && robotDistenceToGoalY < 1) { 
            robotVelocityX = AutoConstants.kMaxAutonSpeedInMetersPerSecond * Math.pow(robotDistenceToGoalX, AutoConstants.kAutoSlowRate) * AutoConstants.kAutoSlowDownSpeed;
            robotVelocityY = AutoConstants.kMaxAutonSpeedInMetersPerSecond * Math.pow(robotDistenceToGoalY, AutoConstants.kAutoSlowRate) * AutoConstants.kAutoSlowDownSpeed;
        } else if(robotDistenceToGoalX < 1 && robotDistenceToGoalY > 1) {
            robotVelocityX = (robotDistenceToGoalX * AutoConstants.kMaxAutonSpeedInMetersPerSecond) / robotDistenceToGoalY;
            robotVelocityY = AutoConstants.kMaxAutonSpeedInMetersPerSecond;
        } else if(robotDistenceToGoalX > 1 && robotDistenceToGoalY < 1) {
            robotVelocityX = AutoConstants.kMaxAutonSpeedInMetersPerSecond;
            robotVelocityY = (robotDistenceToGoalY * AutoConstants.kMaxAutonSpeedInMetersPerSecond) / robotDistenceToGoalX;
        } else if(robotDistenceToGoalX > 1 && robotDistenceToGoalY > 1) {
            if(robotDistenceToGoalX >= robotDistenceToGoalY) {
                robotVelocityX = AutoConstants.kMaxAutonSpeedInMetersPerSecond;
                robotVelocityY = (robotDistenceToGoalY * AutoConstants.kMaxAutonSpeedInMetersPerSecond) / robotDistenceToGoalX;
            } else {
                robotVelocityX = (robotDistenceToGoalX * AutoConstants.kMaxAutonSpeedInMetersPerSecond) / robotDistenceToGoalY;
                robotVelocityY = AutoConstants.kMaxAutonSpeedInMetersPerSecond;
            } 
        } 
        robotVelocityX *= Math.signum(goalPose.getX() - robotPose.getX());
        robotVelocityY *= Math.signum(goalPose.getY() - robotPose.getY());
        return new Translation2d(robotVelocityX, robotVelocityY);
       
    }
  
   
}
