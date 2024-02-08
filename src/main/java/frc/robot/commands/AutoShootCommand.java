// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// booba

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.IntakeCommands.HandOffToShooterCommand;
import frc.robot.commands.ShooterCommands.RampShooterAtDifforentSpeedCommand;
import frc.robot.commands.ShooterCommands.StopShooterCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakeSensorSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.CalculateSpeakerShootingPosition;

public class AutoShootCommand extends Command{
    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private PivotSubsystem m_pivotSubsystem;
    private IntakeSensorSubsystem m_intakeSensorSubsystem;

    private RampShooterAtDifforentSpeedCommand rampShooterCommand;
    private StopShooterCommand stopShooterCommmand;
    private HandOffToShooterCommand handOffToShooterCommand;

    public AutoShootCommand(ShooterSubsystem m_shooterSubsystem, 
    IntakeSubsystem m_intakeSubsystem, PivotSubsystem m_pivotSubsystem, IntakeSensorSubsystem m_intakeSensorSubsystem) {
        this.m_shooterSubsystem = m_shooterSubsystem;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.m_pivotSubsystem = m_pivotSubsystem;
        this.m_intakeSensorSubsystem = m_intakeSensorSubsystem;

        this.rampShooterCommand = new RampShooterAtDifforentSpeedCommand(this.m_shooterSubsystem);
        this.stopShooterCommmand = new StopShooterCommand(this.m_shooterSubsystem);
        this.handOffToShooterCommand = new HandOffToShooterCommand(this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() { 
        SmartDashboard.putString("Shooting Stage", "Scheduled Ramp Shooter Command");
        this.rampShooterCommand = new RampShooterAtDifforentSpeedCommand(this.m_shooterSubsystem);
        this.stopShooterCommmand = new StopShooterCommand(this.m_shooterSubsystem);
        this.handOffToShooterCommand = new HandOffToShooterCommand(this.m_intakeSubsystem, this.m_pivotSubsystem, this.m_intakeSensorSubsystem);
        rampShooterCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("Shooting Hand Off Finished", this.handOffToShooterCommand.isFinished());

        if(!handOffToShooterCommand.isScheduled() && this.rampShooterCommand.isFinished()) {
            SmartDashboard.putString("Shooting Stage", "Scheduled Hand Off to Shooter Command");

            handOffToShooterCommand.schedule();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Shooting Stage", "Scheduled Stop Shooter Command");

        this.stopShooterCommmand.schedule();
        this.handOffToShooterCommand.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.handOffToShooterCommand.isFinished();
    }

}
