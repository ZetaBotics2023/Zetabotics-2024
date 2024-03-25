// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition;

import javax.xml.namespace.QName;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutonConfigurationConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.LEDSubsystem.CTRELEDSubsystem;
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

    private CTRELEDSubsystem m_ledSubsystem;

    private boolean firstLimitCrossed = false;
    private boolean secondLimitCrossed = false;
    private boolean thirdLimitCrossed = false;

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
        this.firstLimitCrossed = false;
        this.secondLimitCrossed = false;
        this.thirdLimitCrossed = false;

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

        } 

        if (BatteryCharge.getAverageVoltage() < AutoConstants.kSecondBatteryPIDLimit && !this.secondLimitCrossed) {
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

        }
        if(BatteryCharge.getAverageVoltage() < AutoConstants.kThirdBatteryPIDLimit && !this.thirdLimitCrossed) {
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
        //SmartDashBoard.putBoolean("First Limit", firstLimitCrossed);
        //SmartDashBoard.putBoolean("Second Limit", secondLimitCrossed);
        //SmartDashBoard.putBoolean("Thrid Limit", secondLimitCrossed);

    }
}
