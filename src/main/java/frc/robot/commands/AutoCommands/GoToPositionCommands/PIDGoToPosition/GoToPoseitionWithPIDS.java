// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.LEDSubsystem.CTRELEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDSubsystem.RGBColor;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;


public class GoToPoseitionWithPIDS extends Command{

    DriveSubsystem m_driveSubsystem;
    Pose2d rbootPose;
    Pose2d goalEndPose;

    private ProfiledPIDController translationXController;
    private ProfiledPIDController translationYController;
    private ProfiledPIDController headingPIDController;

    private CTRELEDSubsystem m_ledSubsystem;

    public boolean highTolorence = false;

    public GoToPoseitionWithPIDS(DriveSubsystem m_driveSubsystem, Pose2d goalEndPose, CTRELEDSubsystem m_ledSubsystem) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.goalEndPose = goalEndPose;

        this.translationXController = new ProfiledPIDController(
            AutoConstants.kTranslationPIDControllerP, AutoConstants.kTranslationPIDControllerI, AutoConstants.kTranslationPIDControllerD, AutoConstants.kTranslationControllerConstraints);
        this.translationXController.setTolerance(AutoConstants.kTranslationPIDControllerPositionalTolerance, AutoConstants.kTranslationPIDControllerVelocityTolerance);
        this.translationXController.setIntegratorRange(-.3, .3);
        this.translationXController.reset(this.m_driveSubsystem.getRobotPose().getX());

        this.translationYController = new ProfiledPIDController(
            AutoConstants.kTranslationPIDControllerP, AutoConstants.kTranslationPIDControllerI, AutoConstants.kTranslationPIDControllerD, AutoConstants.kTranslationControllerConstraints);
        this.translationYController.setTolerance(AutoConstants.kTranslationPIDControllerPositionalTolerance, AutoConstants.kTranslationPIDControllerVelocityTolerance);
        this.translationYController.setIntegratorRange(-.3, .3);
        this.translationYController.reset(this.m_driveSubsystem.getRobotPose().getY());

        this.headingPIDController = new ProfiledPIDController(AutoConstants.kHeadingPIDControllerP, AutoConstants.kHeadingPIDControllerI,
        AutoConstants.kHeadingPIDControllerD, AutoConstants.kThetaControllerConstraints);
        this.headingPIDController.setTolerance(AutoConstants.kHeadingPIDControllerTolerance);
        this.headingPIDController.setIntegratorRange(-0.3, 0.3);
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());

        this.m_ledSubsystem = m_ledSubsystem;

        addRequirements(this.m_ledSubsystem);
    }

    public void initialize() {
        SwerveDriveConstants.driverController = false;
    
        this.translationXController.reset(this.m_driveSubsystem.getRobotPose().getX());
        this.translationYController.reset(this.m_driveSubsystem.getRobotPose().getY());
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());

        this.m_ledSubsystem.setSolidColor(RGBColor.Purple.color);
    }

    public void execute() {
       // this.updateControllersForVoltage();
        this.m_driveSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            this.translationXController.calculate(this.m_driveSubsystem.getRobotPose().getX(), this.goalEndPose.getX()),
            this.translationYController.calculate(this.m_driveSubsystem.getRobotPose().getY(), this.goalEndPose.getY()),
            this.headingPIDController.calculate(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees(), this.goalEndPose.getRotation().getDegrees()),
            this.m_driveSubsystem.getRobotPose().getRotation()
        ));
        
            if(Math.abs(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees() - this.goalEndPose.getRotation().getDegrees()) 
                <= AutoConstants.kHeadingPIDControllerToleranceHigh 
                && Math.abs(this.m_driveSubsystem.getRobotPose().getX() - goalEndPose.getX())
                <= AutoConstants.kTranslationPIDControllerPositionalToleranceHigh &&
                Math.abs(this.m_driveSubsystem.getRobotPose().getY() - goalEndPose.getY())
                <= AutoConstants.kTranslationPIDControllerPositionalToleranceHigh) {
                    
                this.highTolorence = true;
            }
        //SmartDashBoard.putBoolean("High Tolorence", this.highTolorence);
    }

    @Override
    public void end(boolean interrupted) {
        this.m_ledSubsystem.setSolidColor(RGBColor.Orange.color);
        this.m_driveSubsystem.stop();
        SwerveDriveConstants.driverController = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.translationXController.atGoal() && 
            this.translationYController.atGoal() &&
            this.headingPIDController.atGoal();        
        }
}
