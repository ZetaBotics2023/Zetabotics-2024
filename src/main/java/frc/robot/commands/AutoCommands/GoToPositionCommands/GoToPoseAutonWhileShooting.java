// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.AutoCommands.GoToPositionCommands;

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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.IntakeCommands.HandOffToShooterAuton;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.CalculateGoToPoseVelocityAuton;
import frc.robot.utils.CalculateSpeakerShootingPosition;
import frc.robot.utils.InTeleop;


public class GoToPoseAutonWhileShooting extends Command{
    private DriveSubsystem m_driveSubsystem;
    private Pose2d goalEndPose;
    private HandOffToShooterAuton handOfftoShooterAuton;
    private ProfiledPIDController headingPIDController;
    private final SlewRateLimiter translationXLimiter;
    private final SlewRateLimiter translationYLimiter;  

    private double startingRobotDistenceFromPose = 1000000;
    private double persentToPose;


    public GoToPoseAutonWhileShooting(DriveSubsystem m_driveSubsystem, Pose2d goalEndPose, HandOffToShooterAuton handOffToShooterAuton, double shootDistence) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.goalEndPose = goalEndPose;
        this.handOfftoShooterAuton = handOffToShooterAuton;
        this.headingPIDController = new ProfiledPIDController(AutoConstants.kHeadingPIDControllerAutoP, AutoConstants.kHeadingPIDControllerAutoI,
        AutoConstants.kHeadingPIDControllerAutoD, AutoConstants.kThetaControllerConstraintsAuto);
        this.headingPIDController.setTolerance(AutoConstants.kHeadingPIDControllerToleranceAuto);
        this.headingPIDController.setIntegratorRange(-0.3, 0.3);
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
        this.translationXLimiter = new SlewRateLimiter(20);
        this.translationYLimiter = new SlewRateLimiter(20);
        this.persentToPose = shootDistence;
        addRequirements(m_driveSubsystem);
    }

    public void initialize() {
        this.startingRobotDistenceFromPose = Math.sqrt(Math.pow(this.m_driveSubsystem.getRobotPose().getX() + this.m_driveSubsystem.getRobotPose().getY(), 2))
         - Math.sqrt(Math.pow(this.goalEndPose.getX() + this.goalEndPose.getY(), 2));

        this.translationXLimiter.reset(0);
        this.translationYLimiter.reset(0);
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
        this.handOfftoShooterAuton.schedule();
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

        SmartDashboard.putNumber("Auton Goal Thata", this.goalEndPose.getRotation().getDegrees());
        Translation2d robotTransformVelocity = CalculateGoToPoseVelocityAuton.calculateGoToPoseVelocity(m_driveSubsystem.getRobotPose(), this.goalEndPose);
        SmartDashboard.putNumber("Goal X Vel", robotTransformVelocity.getX());
        SmartDashboard.putNumber("Goal Y Vel", robotTransformVelocity.getY());
        this.m_driveSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
        this.translationXLimiter.calculate(robotTransformVelocity.getX()),
        this.translationYLimiter.calculate(robotTransformVelocity.getY()),
        headingPIDController.calculate(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees(), targetAngle),
        this.m_driveSubsystem.getRobotPose().getRotation()));

        double currentDistenceToEndPose = Math.sqrt(Math.pow(this.m_driveSubsystem.getRobotPose().getX() + this.m_driveSubsystem.getRobotPose().getY(), 2))
         - Math.sqrt(Math.pow(this.goalEndPose.getX() + this.goalEndPose.getY(), 2));

         if(this.persentToPose * startingRobotDistenceFromPose < currentDistenceToEndPose && !this.handOfftoShooterAuton.isScheduled()) {
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
        boolean isInPosition = Math.abs(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees() - this.goalEndPose.getRotation().getDegrees()) <= AutoConstants.kHeadingPIDControllerTolerance && Math.abs(this.m_driveSubsystem.getRobotPose().getX() - goalEndPose.getX()) <= AutoConstants.kAutoPositonTolorenceAuto &&
         Math.abs(this.m_driveSubsystem.getRobotPose().getY() - goalEndPose.getY()) <= AutoConstants.kAutoPositonTolorenceAuto;
         return isInPosition && this.handOfftoShooterAuton.isFinished();
    }
}
