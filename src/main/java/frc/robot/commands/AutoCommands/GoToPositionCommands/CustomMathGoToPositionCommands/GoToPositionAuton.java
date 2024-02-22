// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.AutoCommands.GoToPositionCommands.CustomMathGoToPositionCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.CalculateGoToPoseVelocityAuton;


public class GoToPositionAuton extends Command{
    DriveSubsystem m_driveSubsystem;
    Pose2d goalEndPose;
    private ProfiledPIDController headingPIDController;
    private final SlewRateLimiter translationXLimiter;
    private final SlewRateLimiter translationYLimiter;

    
    
    public GoToPositionAuton(DriveSubsystem m_driveSubsystem, Pose2d goalEndPose) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.goalEndPose = goalEndPose;
        this.headingPIDController = new ProfiledPIDController(AutoConstants.kHeadingPIDControllerAutoP, AutoConstants.kHeadingPIDControllerAutoI,
        AutoConstants.kHeadingPIDControllerAutoD, AutoConstants.kThetaControllerConstraintsAuto);
        this.headingPIDController.setTolerance(AutoConstants.kHeadingPIDControllerToleranceAuto);
        this.headingPIDController.setIntegratorRange(-0.3, 0.3);
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
        this.translationXLimiter = new SlewRateLimiter(20);
        this.translationYLimiter = new SlewRateLimiter(20);
        addRequirements(m_driveSubsystem);
    }

    public void initialize() {
        this.translationXLimiter.reset(0);
        this.translationYLimiter.reset(0);
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
    }

    public void execute() {
        SmartDashboard.putNumber("Auton Goal Thata", this.goalEndPose.getRotation().getDegrees());
        Translation2d robotTransformVelocity = CalculateGoToPoseVelocityAuton.calculateGoToPoseVelocity(m_driveSubsystem.getRobotPose(), this.goalEndPose);
        SmartDashboard.putNumber("Goal X Vel", robotTransformVelocity.getX());
        SmartDashboard.putNumber("Goal Y Vel", robotTransformVelocity.getY());
        this.m_driveSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
        this.translationXLimiter.calculate(robotTransformVelocity.getX()),
        this.translationYLimiter.calculate(robotTransformVelocity.getY()),
        headingPIDController.calculate(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees(), this.goalEndPose.getRotation().getDegrees()),
        this.m_driveSubsystem.getRobotPose().getRotation()
        ));
    }

    @Override
    public void end(boolean interrupted) {
        this.m_driveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees() - this.goalEndPose.getRotation().getDegrees()) 
        <= AutoConstants.kHeadingPIDControllerTolerance && Math.abs(this.m_driveSubsystem.getRobotPose().getX() - goalEndPose.getX()) <= AutoConstants.kAutoPositonToleranceAuto &&
         Math.abs(this.m_driveSubsystem.getRobotPose().getY() - goalEndPose.getY()) <= AutoConstants.kAutoPositonToleranceAuto;
    }
}
