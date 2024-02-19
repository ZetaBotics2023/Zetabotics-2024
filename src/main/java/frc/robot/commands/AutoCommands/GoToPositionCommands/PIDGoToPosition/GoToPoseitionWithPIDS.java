// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;


public class GoToPoseitionWithPIDS extends Command{

    DriveSubsystem m_driveSubsystem;
    Pose2d rbootPose;
    Pose2d goalEndPose;

    private ProfiledPIDController translationXController;
    private ProfiledPIDController translationYController;
    private ProfiledPIDController headingPIDController;
    
    public GoToPoseitionWithPIDS(DriveSubsystem m_driveSubsystem, Pose2d goalEndPose) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.goalEndPose = goalEndPose;

        this.translationXController = new ProfiledPIDController(
            AutoConstants.kTranslationAutoPIDControllerP, AutoConstants.kTranslationAutoPIDControllerI, AutoConstants.kTranslationAutoPIDControllerD, AutoConstants.kTranslationAutoControllerConstraints);
        this.translationXController.setTolerance(AutoConstants.kTranslationAutoPIDControllerPositionalTolerance, AutoConstants.kTranslationAutoPIDControllerVelocityTolerance);
        this.translationXController.setIntegratorRange(-.3, .3);
        this.translationXController.reset(this.m_driveSubsystem.getRobotPose().getX());

        this.translationYController = new ProfiledPIDController(
            AutoConstants.kTranslationAutoPIDControllerP, AutoConstants.kTranslationAutoPIDControllerI, AutoConstants.kTranslationAutoPIDControllerD, AutoConstants.kTranslationAutoControllerConstraints);
        this.translationYController.setTolerance(AutoConstants.kTranslationAutoPIDControllerPositionalTolerance, AutoConstants.kTranslationAutoPIDControllerVelocityTolerance);
        this.translationYController.setIntegratorRange(-.3, .3);
        this.translationYController.reset(this.m_driveSubsystem.getRobotPose().getY());

        this.headingPIDController = new ProfiledPIDController(AutoConstants.kHeadingPIDControllerAutoP, AutoConstants.kHeadingPIDControllerAutoI,
        AutoConstants.kHeadingPIDControllerAutoD, AutoConstants.kThetaControllerConstraintsAuto);
        this.headingPIDController.setTolerance(AutoConstants.kHeadingPIDControllerToleranceAuto);
        this.headingPIDController.setIntegratorRange(-0.3, 0.3);
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());

        addRequirements(m_driveSubsystem);
    }

    public void initialize() {
        this.translationXController.reset(this.m_driveSubsystem.getRobotPose().getX());
        this.translationYController.reset(this.m_driveSubsystem.getRobotPose().getY());
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
    }

    public void execute() {
        this.m_driveSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            this.translationXController.calculate(this.m_driveSubsystem.getRobotPose().getX(), this.goalEndPose.getX()),
            this.translationYController.calculate(this.m_driveSubsystem.getRobotPose().getY(), this.goalEndPose.getY()),
            this.headingPIDController.calculate(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees(), this.goalEndPose.getRotation().getDegrees()),
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
        return Math.abs(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees() - this.goalEndPose.getRotation().getDegrees()) <= AutoConstants.kHeadingPIDControllerTolerance && Math.abs(this.m_driveSubsystem.getRobotPose().getX() - goalEndPose.getX()) <= AutoConstants.kAutoPositonTolorenceAuto &&
         Math.abs(this.m_driveSubsystem.getRobotPose().getY() - goalEndPose.getY()) <= AutoConstants.kAutoPositonTolorenceAuto;
    }
}
