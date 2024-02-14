// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.AutoCommands;

import java.nio.file.DirectoryStream;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.CalculateGoToPoseVelocity;
import frc.robot.utils.InTeleop;


public class GoToPosition extends Command{
    DriveSubsystem m_driveSubsystem;
    TurnToAngle m_turnToAngle;
    Pose2d goalEndPose;
    private ProfiledPIDController headingPIDController;
    private final SlewRateLimiter translationXLimiter;
    private final SlewRateLimiter translationYLimiter;

    
    
    public GoToPosition(DriveSubsystem m_driveSubsystem, Pose2d goalEndPose) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.goalEndPose = goalEndPose;
        this.headingPIDController = new ProfiledPIDController(SwerveDriveConstants.kHeadingPIDControllerP, SwerveDriveConstants.kHeadingPIDControllerI,
        SwerveDriveConstants.kHeadingPIDControllerD, SwerveDriveConstants.kThetaControllerConstraints);
        this.headingPIDController.setTolerance(SwerveDriveConstants.kHeadingPIDControllerTolerance);
        this.headingPIDController.setIntegratorRange(-0.3, 0.3);
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
        this.translationXLimiter = new SlewRateLimiter(20);
        this.translationYLimiter = new SlewRateLimiter(20);
        addRequirements(m_driveSubsystem);
    }

    public void initialize() {
        SmartDashboard.putBoolean("GO TO POSE TRIGGURED", true);
        this.translationXLimiter.reset(0);
        this.translationYLimiter.reset(0);
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
    }

    public void execute() {
        SmartDashboard.putNumber("Auton Goal Thata", this.goalEndPose.getRotation().getDegrees());
        Translation2d robotTransformVelocity = CalculateGoToPoseVelocity.calculateGoToPoseVelocity(m_driveSubsystem.getRobotPose(), this.goalEndPose);
        SmartDashboard.putNumber("Goal X Vel", robotTransformVelocity.getX());
        SmartDashboard.putNumber("Goal Y Vel", robotTransformVelocity.getY());
        this.m_driveSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
        this.translationXLimiter.calculate(robotTransformVelocity.getX()),
        this.translationYLimiter.calculate(robotTransformVelocity.getY()),
        headingPIDController.calculate(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees(), this.goalEndPose.getRotation().getDegrees()),
        this.m_driveSubsystem.getRobotPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        this.m_driveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees() - this.goalEndPose.getRotation().getDegrees()) <= 2 && Math.abs(this.m_driveSubsystem.getRobotPose().getX() - goalEndPose.getX()) <= SwerveDriveConstants.kAutoPositonTolorence && Math.abs(this.m_driveSubsystem.getRobotPose().getY() - goalEndPose.getY()) <= SwerveDriveConstants.kAutoPositonTolorence;
    }
}
