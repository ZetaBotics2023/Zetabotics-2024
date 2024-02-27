// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoShootCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConfigurationConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition.GoToPoseitionWithPIDS;
import frc.robot.commands.IntakeCommands.HandOffToShooterCommand;
import frc.robot.commands.ShooterCommands.RampShooterAtDifforentSpeedCommand;
import frc.robot.commands.ShooterCommands.StopShooterCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.CalculateSpeakerShootingPosition;
import frc.robot.utils.MirrablePose2d;

public class AutoShootPositionRightCommand extends Command{
    private DriveSubsystem m_driveSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private PivotSubsystem m_pivotSubsystem;
    private IntakeSensorSubsystem m_intakeSensorSubsystem;

    private RampShooterAtDifforentSpeedCommand rampShooterCommand;
    private StopShooterCommand stopShooterCommmand;
    private HandOffToShooterCommand handOffToShooterCommand;


    private GoToPoseitionWithPIDS goToPosition;

    public AutoShootPositionRightCommand(DriveSubsystem m_driveSubsystem, ShooterSubsystem m_shooterSubsystem, 
    IntakeSubsystem m_intakeSubsystem, PivotSubsystem m_pivotSubsystem, IntakeSensorSubsystem m_intakeSensorSubsystem) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.m_shooterSubsystem = m_shooterSubsystem;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.m_pivotSubsystem = m_pivotSubsystem;
        this.m_intakeSensorSubsystem = m_intakeSensorSubsystem;

        this.rampShooterCommand = new RampShooterAtDifforentSpeedCommand(this.m_shooterSubsystem);
        this.stopShooterCommmand = new StopShooterCommand(this.m_shooterSubsystem);
        this.handOffToShooterCommand = new HandOffToShooterCommand(this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);
        this.goToPosition = new GoToPoseitionWithPIDS(m_driveSubsystem, new Pose2d(1000.0, 1000.0, new Rotation2d()));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        this.rampShooterCommand = new RampShooterAtDifforentSpeedCommand(this.m_shooterSubsystem);
        this.stopShooterCommmand = new StopShooterCommand(this.m_shooterSubsystem);
        this.handOffToShooterCommand = new HandOffToShooterCommand(this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);

        MirrablePose2d shootingPose = AutonConfigurationConstants.kIsBlueAlliance ? ShooterConstants.kRightShootingPose : ShooterConstants.kLeftShootingPose;
        this.goToPosition = new GoToPoseitionWithPIDS(m_driveSubsystem, shootingPose.getPose(!AutonConfigurationConstants.kIsBlueAlliance));

        goToPosition.schedule();
        rampShooterCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!handOffToShooterCommand.isScheduled() && this.rampShooterCommand.isFinished() && this.goToPosition.isFinished()) {
            handOffToShooterCommand.schedule();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.rampShooterCommand.cancel();
        this.handOffToShooterCommand.cancel();
        this.goToPosition.cancel();
        this.stopShooterCommmand.schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;//this.handOffToShooterCommand.isFinished();
    }

}