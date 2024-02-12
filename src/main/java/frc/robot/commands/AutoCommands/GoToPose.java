// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.AutoCommands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;

public class GoToPose {
    public static Command goToPose(Pose2d startLocation, Pose2d endLocaiton) {
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      startLocation,
      endLocaiton);

      // Create the path using the bezier points created above
      PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(AutoConstants.kMaxAutonSpeedInMetersPerSecond, AutoConstants.kMaxAutonAccelerationInMetersPerSecondSqr,
          AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared), 
          new GoalEndState(0.0, endLocaiton.getRotation())
      );
      return AutoBuilder.followPath(path);
      
}
}