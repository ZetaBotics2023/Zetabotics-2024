// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.IntakeCommands.HandOffToShooterAuton;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.MirrablePose2d;


public class GoToPoseAutonWhileShootingWithPIDS extends Command{
    private DriveSubsystem m_driveSubsystem;
    private Pose2d goalEndPose;
    private HandOffToShooterAuton handOfftoShooterAuton;

    private double startingRobotDistenceFromPose = 1000000;
    private double percentToPose;

    private ProfiledPIDController translationXController;
    private ProfiledPIDController translationYController;
    private ProfiledPIDController headingPIDController;


    public GoToPoseAutonWhileShootingWithPIDS(DriveSubsystem m_driveSubsystem, HandOffToShooterAuton handOffToShooterAuton,
     Pose2d goalEndPose, double percentToPose) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.goalEndPose = goalEndPose;
        this.handOfftoShooterAuton = handOffToShooterAuton;

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

        this.percentToPose = percentToPose;
        addRequirements(m_driveSubsystem);

    }

    public void initialize() {
        this.translationXController.reset(this.m_driveSubsystem.getRobotPose().getX());
        this.translationYController.reset(this.m_driveSubsystem.getRobotPose().getY());
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
        
        double robotPoseX = this.m_driveSubsystem.getRobotPose().getX();
        double robotPoseY = this.m_driveSubsystem.getRobotPose().getY();
        double distenceFromPoseX = robotPoseX - this.goalEndPose.getX();
        double distanceFronPoseY = robotPoseY - this.goalEndPose.getY();
        this.startingRobotDistenceFromPose = Math.sqrt(Math.pow(distenceFromPoseX + distanceFronPoseY, 2));
        }

    public void execute() {

        Pose2d aprilTagPosition = (
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
            VisionConstants.kBlueAllianceShooterAprilTagPosition : VisionConstants.kRedAllianceShooterAprilTagPosition);

        double robotPoseXBefore = this.m_driveSubsystem.getRobotPose().getX();
        double robotPoseYBefore = this.m_driveSubsystem.getRobotPose().getY();

        // Calculate the distance between the robot and AprilTag in X and Y
        // These have been kept here as a comment in case we need them later, but they aren't used.
        // This is because they were for calculating our target angle, which is now done by a simple prediction below.
        //double distanceToAprilTagX = robotPoseXBefore - aprilTagPosition.getX();
        //double distanceToAprilTagY = robotPoseYBefore - aprilTagPosition.getY();
        //double distanceToAprilTag = Math.sqrt(Math.pow(distanceToAprilTagX, 2) + Math.pow(distanceToAprilTagY, 2));

        Translation2d robotTransformVelocity = new Translation2d(
            this.translationXController.calculate(this.m_driveSubsystem.getRobotPose().getX(), this.goalEndPose.getX()),
            this.translationYController.calculate(this.m_driveSubsystem.getRobotPose().getY(), this.goalEndPose.getY())
        );

        //SmartDashBoard.putNumber("Goal X Vel S", robotTransformVelocity.getX());
        //SmartDashBoard.putNumber("Goal Y Vel S", robotTransformVelocity.getY());

        // We need to predict what our position relative to the AprilTag will be in the time it takes to shoot
        // To get a halfway-decent estimate, we can use the distance formula d=v/t
        // Our velocity will be the robot's current velocity, while the time will be our shooter time
        // Using this, we calculate the distance we will have gone in the shooter time, and add it to the current position
        double robotPoseAfterShootTimeX = robotPoseXBefore + (robotTransformVelocity.getX() / ShooterConstants.kShootTimeAuto);
        double robotPoseAfterShootTimeY = robotPoseYBefore + (robotTransformVelocity.getY() / ShooterConstants.kShootTimeAuto);

        // Now find the distance to the AprilTag using the predicted values
        double distanceToAprilTagAfterShootTimeX = robotPoseAfterShootTimeX - aprilTagPosition.getX();
        double distanceToAprilTagAfterShootTimeY = robotPoseAfterShootTimeY - aprilTagPosition.getY();
        double distanceToAprilTagAfterShootTime = Math.sqrt(Math.pow(distanceToAprilTagAfterShootTimeX, 2) + Math.pow(distanceToAprilTagAfterShootTimeY, 2));

        double targetAngle = 0;
        if(Math.abs(robotPoseAfterShootTimeX - aprilTagPosition.getY()) >= .25) {
            targetAngle = Units.radiansToDegrees(Math.asin(distanceToAprilTagAfterShootTimeY/distanceToAprilTagAfterShootTime));
        }

        //SmartDashBoard.putNumber("Auton Goal Theta S", targetAngle);


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
        AutoConstants.kHeadingPIDControllerTolerance &&
        Math.abs(this.m_driveSubsystem.getRobotPose().getX() - goalEndPose.getX()) <=
        AutoConstants.kAutoPositonToleranceAuto &&
        Math.abs(this.m_driveSubsystem.getRobotPose().getY() - goalEndPose.getY()) <=
        AutoConstants.kAutoPositonToleranceAuto;

        return isInPosition && this.handOfftoShooterAuton.isFinished();
    }
}