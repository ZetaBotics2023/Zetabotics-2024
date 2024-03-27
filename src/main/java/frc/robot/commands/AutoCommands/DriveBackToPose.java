package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class DriveBackToPose extends Command{
    private double pointX;
    private DriveSubsystem m_driveSubsystem;
    public DriveBackToPose(DriveSubsystem m_driveSubsystem,double pointX) {
        this.pointX = pointX;
        this.m_driveSubsystem = m_driveSubsystem;
    }

    public boolean isFinished() {
        this.m_driveSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
            4.3,
            0,
            0,
            this.m_driveSubsystem.getRobotPose().getRotation()
        ));
        return this.m_driveSubsystem.getRobotPose().getX() >= pointX;
    }
}
