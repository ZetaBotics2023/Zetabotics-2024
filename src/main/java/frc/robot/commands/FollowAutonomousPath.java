// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// booba

package frc.robot.commands;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class FollowAutonomousPath {

  // Assuming this is a method in your drive subsystem
public static Command followPathCommand(DriveSubsystem driveSubsystem, String pathName){

    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work

    /*return new FollowPathWithEvents(
        new FollowPathHolonomic(
          path,
          poseEstimatorSubsystem::getCurrentPose,
          driveSubsystem::getChassisSpeeds,
          driveSubsystem::drive,
          new PIDConstants(0), // TODO: Add real PID constants
          new PIDConstants(0),
          Constants.AutoConstants.kMaxAutonSpeedInMetersPerSecond,
          Constants.SwerveDriveConstants.kRadiusFromCenterToSwerveDrives,
          new ReplanningConfig(),
          driveSubsystem
        ),
        path, 
        poseEstimatorSubsystem::getCurrentPose
    );*/
    return new Command() {
      
    };
}

// Subtracts every point's translation from the dimensions of the field, resulting in a bidirectionally-mirrored path
public static PathPlannerPath reflectPath(PathPlannerPath original) {
  List<PathPoint> points = original.getAllPathPoints();
  List<PathPoint> newPoints = new ArrayList<PathPoint>();
  points.forEach((PathPoint point) -> {
    point = new PathPoint(new Translation2d(Constants.FieldConstants.kWidth - point.position.getX(), Constants.FieldConstants.kLength - point.position.getY()), point.rotationTarget);
    newPoints.add(point);
  });
  PathPlannerPath newPath = PathPlannerPath.fromPathPoints(newPoints, original.getGlobalConstraints(), original.getGoalEndState());
  return newPath;
}



// SUbtracts every
public static PathPlannerPath rotatePath(PathPlannerPath original) {
  List<PathPoint> points = original.getAllPathPoints();
  List<PathPoint> newPoints = new ArrayList<PathPoint>();
  points.forEach((PathPoint point) -> {
    point = new PathPoint(new Translation2d(Constants.FieldConstants.kWidth - point.position.getX(), point.position.getY()), point.rotationTarget);
    newPoints.add(point);
  });
  PathPlannerPath newPath = PathPlannerPath.fromPathPoints(newPoints, original.getGlobalConstraints(), original.getGoalEndState());
  return newPath;
}

}