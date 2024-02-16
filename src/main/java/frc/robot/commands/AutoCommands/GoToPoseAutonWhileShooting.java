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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.IntakeCommands.HandOffToShooterAuton;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.CalculateGoToPoseVelocityAuton;
import frc.robot.utils.CalculateSpeakerShootingPosition;
import frc.robot.utils.InTeleop;
import frc.robot.utils.MirrablePose2d;


public class GoToPoseAutonWhileShooting extends Command{
    private DriveSubsystem m_driveSubsystem;
    private Pose2d goalEndPose;
    private HandOffToShooterAuton handOfftoShooterAuton;
    private ProfiledPIDController headingPIDController;
    private SlewRateLimiter translationXLimiter;
    private SlewRateLimiter translationYLimiter;  

    private double startingRobotDistenceFromPose = 1000000;
    private double percentToPose;

    public GoToPoseAutonWhileShooting(DriveSubsystem m_driveSubsystem, HandOffToShooterAuton handOffToShooterAuton,
     MirrablePose2d goalEndPose, double percentToPose) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.goalEndPose = goalEndPose;
        this.handOfftoShooterAuton = handOffToShooterAuton;
        this.headingPIDController = new ProfiledPIDController(SwerveDriveConstants.kHeadingPIDControllerAutoP, SwerveDriveConstants.kHeadingPIDControllerAutoI,
        SwerveDriveConstants.kHeadingPIDControllerAutoD, SwerveDriveConstants.kThetaControllerConstraintsAuto);
        this.headingPIDController.setTolerance(SwerveDriveConstants.kHeadingPIDControllerToleranceAuto);
        this.headingPIDController.setIntegratorRange(-0.3, 0.3);
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
        this.percentToPose = percentToPose;
        addRequirements(m_driveSubsystem);

    }

    public void initialize() {
        double robotPoseX = this.m_driveSubsystem.getRobotPose().getX();
        double robotPoseY = this.m_driveSubsystem.getRobotPose().getY();
        double distenceFromPoseX = robotPoseX - this.goalEndPose.getX();
        double distanceFronPoseY = robotPoseY - this.goalEndPose.getY();
        this.startingRobotDistenceFromPose = Math.sqrt(Math.pow(distenceFromPoseX + distanceFronPoseY, 2));

        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
    }

    public void execute() {

        Pose2d aprilTagPosition = (
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
            VisionConstants.kBlueAllianceShooterAprilTagPosition : VisionConstants.kRedAllianceShooterAprilTagPosition);

        // Calculate the distance between the robot and AprilTag in X and Y
        double distanceToAprilTagX = this.m_driveSubsystem.getRobotPose().getX() - aprilTagPosition.getX();
        double distanceToAprilTagY = this.m_driveSubsystem.getRobotPose().getY() - aprilTagPosition.getY();
        double distanceToAprilTag = Math.sqrt(Math.pow(distanceToAprilTagX, 2) + Math.pow(distanceToAprilTagY, 2));
        
        double targetAngle = 0;
        if(Math.abs(this.m_driveSubsystem.getRobotPose().getY() - aprilTagPosition.getY()) >= .25) {
            targetAngle = Units.radiansToDegrees(Math.asin(distanceToAprilTagY/distanceToAprilTag));
        }

        Translation2d robotTransformVelocity = CalculateGoToPoseVelocityAuton.calculateGoToPoseVelocity(m_driveSubsystem.getRobotPose(), this.goalEndPose);

        SmartDashboard.putNumber("Auton Goal Thata S", targetAngle);
        SmartDashboard.putNumber("Goal X Vel S", robotTransformVelocity.getX());
        SmartDashboard.putNumber("Goal Y Vel S", robotTransformVelocity.getY());

        this.m_driveSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                robotTransformVelocity.getX(),
                robotTransformVelocity.getY(),
                headingPIDController.calculate(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees(), targetAngle),
                this.m_driveSubsystem.getRobotPose().getRotation()));

        double robotPoseX = this.m_driveSubsystem.getRobotPose().getX();
        double robotPoseY = this.m_driveSubsystem.getRobotPose().getY();
        double distenceFromPoseX = robotPoseX - this.goalEndPose.getX();
        double distanceFronPoseY = robotPoseY - this.goalEndPose.getY();
        double currentDistenceToEndPose = Math.sqrt(Math.pow(distenceFromPoseX + distanceFronPoseY, 2));
        
        if(startingRobotDistenceFromPose * (1-this.percentToPose) > currentDistenceToEndPose && !this.handOfftoShooterAuton.isScheduled()) {
          this.handOfftoShooterAuton.schedule();
         }
        
    }

    @Override
    public void end(boolean interrupted) {
        this.m_driveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean isInPosition = 
        Math.abs(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees() - 
        this.goalEndPose.getRotation().getDegrees()) <= 
        SwerveDriveConstants.kHeadingPIDControllerTolerance &&
        Math.abs(this.m_driveSubsystem.getRobotPose().getX() - goalEndPose.getX()) <=
        SwerveDriveConstants.kAutoPositonTolorenceAuto &&
        Math.abs(this.m_driveSubsystem.getRobotPose().getY() - goalEndPose.getY()) <=
        SwerveDriveConstants.kAutoPositonTolorenceAuto;

        return isInPosition && this.handOfftoShooterAuton.isFinished();
    }
}
