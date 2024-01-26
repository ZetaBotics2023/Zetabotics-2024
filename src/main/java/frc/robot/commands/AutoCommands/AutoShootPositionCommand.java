// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// booba

package frc.robot.commands.AutoCommands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class AutoShootPositionCommand extends Command{
    private DriveSubsystem m_driveSubsystem;
    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem = m_intakeSubsystem;
    private PivotSubsystem = m_pivotSubsystem;
    private IntakeSensorSubsystem = m_intakeSensorSubsystem;

    private Command goToShootPosition;



    public AutoShootPositionCommand(DriveSubsystem m_driveSubsystem, ShooterSubsystem m_shooterSubsystem, 
    IntakeSubsystem m_intakeSubsystem, PivotSubsystem m_pivotSubsystem, IntakeSensorSubsystem m_intakeSensorSubsystem) {
        this.m_driveSubsystem = m_driveSubsystem;
        this.m_shooterSubsystem = m_shooterSubsystem;
        this.m_intakeSubsystem = m)ShooterSubsystem;
        this.m_pivotSubsystem = m_pivotSubsystem;
        this.m_intakeSensorSubsystem = m_intakeSensorSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        // Start shooter right away
        Pose2d startingPose = this.m_driveSubsystem.getRobotPose();
        Pose2d shootingPosition = CalculateSpeakerShootingPosition.calculateTargetPosition(startingPose);
        goToShootPosition = GoToPose.goToPose(startingPose, shootingPosition);
        goToShootPosition.schedule();    
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        HandOffToShooterCommand handOffToShooterCommand = new HandOffToShooterCommand(this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);
        handOffToShooterCommand.schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.goToShootPosition.isFinished();
    }

}
