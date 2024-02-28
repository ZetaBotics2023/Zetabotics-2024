// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutonConfigurationConstants;
import frc.robot.subsystems.LEDSubsystem.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDSubsystem.RGBColor;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.BatteryCharge;


public class GoToPoseitionWithPIDS extends Command{

    DriveSubsystem m_driveSubsystem;
    Pose2d rbootPose;
    Pose2d goalEndPose;

    private ProfiledPIDController translationXController;
    private ProfiledPIDController translationYController;
    private ProfiledPIDController headingPIDController;
    private LEDSubsystem m_ledSubsystem;

    private boolean firstLimitCrossed = false;
    private boolean secondLimitCrossed = false;
    private boolean thirdLimitCrossed = false;

    public GoToPoseitionWithPIDS(DriveSubsystem m_driveSubsystem, Pose2d goalEndPose, LEDSubsystem m_ledSubsystem) {
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

        addRequirements(m_driveSubsystem);
    }

    public void initialize() {
        this.translationXController.reset(this.m_driveSubsystem.getRobotPose().getX());
        this.translationYController.reset(this.m_driveSubsystem.getRobotPose().getY());
        this.headingPIDController.reset(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees());
    }

    public void execute() {
        this.updateControllersForVoltage();
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
        this.m_ledSubsystem.setSolidColor(RGBColor.Yellow.color);
        this.m_driveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(this.m_driveSubsystem.getRobotPose().getRotation().getDegrees() - this.goalEndPose.getRotation().getDegrees()) <= AutoConstants.kHeadingPIDControllerTolerance && Math.abs(this.m_driveSubsystem.getRobotPose().getX() - goalEndPose.getX()) <= AutoConstants.kAutoPositonToleranceAuto &&
         Math.abs(this.m_driveSubsystem.getRobotPose().getY() - goalEndPose.getY()) <= AutoConstants.kAutoPositonToleranceAuto;
    }

    public void updateControllersForVoltage() {
        if(BatteryCharge.getAverageVoltage() < AutoConstants.kFirstBatteryPIDLimit && !this.firstLimitCrossed) {
            this.firstLimitCrossed = true;

            this.translationXController.setP(AutoConstants.kTranslationPIDControllerPFirstBatteryPIDLimit);
            this.translationXController.setI(AutoConstants.kTranslationPIDControllerIFirstBatteryPIDLimit);
            this.translationXController.setD(AutoConstants.kTranslationPIDControllerDFirstBatteryPIDLimit);

            this.translationYController.setP(AutoConstants.kTranslationPIDControllerPFirstBatteryPIDLimit);
            this.translationYController.setI(AutoConstants.kTranslationPIDControllerIFirstBatteryPIDLimit);
            this.translationYController.setD(AutoConstants.kTranslationPIDControllerDFirstBatteryPIDLimit);

            this.headingPIDController.setP(AutoConstants.kHeadingPIDControllerPFirstBatteryPIDLimit);
            this.headingPIDController.setI(AutoConstants.kHeadingPIDControllerIFirstBatteryPIDLimit);
            this.headingPIDController.setD(AutoConstants.kHeadingPIDControllerDFirstBatteryPIDLimit);

        } else if(BatteryCharge.getAverageVoltage() < AutoConstants.kFirstBatteryPIDLimit && !this.secondLimitCrossed) {
            this.secondLimitCrossed = true;

            this.translationXController.setP(AutoConstants.kTranslationPIDControllerPSecondBatteryPIDLimit);
            this.translationXController.setI(AutoConstants.kTranslationPIDControllerISecondBatteryPIDLimit);
            this.translationXController.setD(AutoConstants.kTranslationPIDControllerDSecondBatteryPIDLimit);

            this.translationYController.setP(AutoConstants.kTranslationPIDControllerPSecondBatteryPIDLimit);
            this.translationYController.setI(AutoConstants.kTranslationPIDControllerISecondBatteryPIDLimit);
            this.translationYController.setD(AutoConstants.kTranslationPIDControllerDSecondBatteryPIDLimit);

            this.headingPIDController.setP(AutoConstants.kHeadingPIDControllerPSecondBatteryPIDLimit);
            this.headingPIDController.setI(AutoConstants.kHeadingPIDControllerISecondBatteryPIDLimit);
            this.headingPIDController.setD(AutoConstants.kHeadingPIDControllerDSecondBatteryPIDLimit);

        } else if(BatteryCharge.getAverageVoltage() < AutoConstants.kThirdBatteryPIDLimit && !this.thirdLimitCrossed) {
            this.thirdLimitCrossed = true;

            this.translationXController.setP(AutoConstants.kTranslationPIDControllerPThirdBatteryPIDLimit);
            this.translationXController.setI(AutoConstants.kTranslationPIDControllerIThirdBatteryPIDLimit);
            this.translationXController.setD(AutoConstants.kTranslationPIDControllerDThirdBatteryPIDLimit);

            this.translationYController.setP(AutoConstants.kTranslationPIDControllerPThirdBatteryPIDLimit);
            this.translationYController.setI(AutoConstants.kTranslationPIDControllerIThirdBatteryPIDLimit);
            this.translationYController.setD(AutoConstants.kTranslationPIDControllerDThirdBatteryPIDLimit);

            this.headingPIDController.setP(AutoConstants.kHeadingPIDControllerPThirdBatteryPIDLimit);
            this.headingPIDController.setI(AutoConstants.kHeadingPIDControllerIThirdBatteryPIDLimit);
            this.headingPIDController.setD(AutoConstants.kHeadingPIDControllerDThirdBatteryPIDLimit);

            this.translationXController.setConstraints(AutoConstants.kTranslationControllerConstraintsLowVoltage);
            this.translationYController.setConstraints(AutoConstants.kTranslationControllerConstraintsLowVoltage);
            this.headingPIDController.setConstraints(AutoConstants.kThetaControllerConstraintsLowVoltage);
        }
    }
}
