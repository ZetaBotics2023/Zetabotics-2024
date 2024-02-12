// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// booba

package frc.robot.commands.AutoCommands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class GoToPose {
    public static Command goToPose(Pose2d startLocation, Pose2d endLocation) {
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(new Translation2d(startLocation.getX(), startLocation.getY()), new Rotation2d(0)),
      new Pose2d(new Translation2d(endLocation.getX(), endLocation.getY()), new Rotation2d(0)));

      // Create the path using the bezier points created above
      PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(AutoConstants.kMaxAutonSpeedInMetersPerSecond, AutoConstants.kMaxAutonAccelerationInMetersPerSecondSqr,
          AutoConstants.kMaxAutonAngulerSpeedInMetersPerSecond, AutoConstants.kMaxAutonAngulerAccelerationInMetersPerSecondSqr), 
          new GoalEndState(0.0, endLocation.getRotation())
      );

    //SmartDashboard.putNumber("Goal X Pose Auto", path.getPoint(path.numPoints()-1).position.getX());
    SmartDashboard.putNumber("Gaol Rotation", path.getPoint(path.numPoints()-1).rotationTarget.getTarget().getDegrees());
    

      return AutoBuilder.followPath(path);
      
}
}