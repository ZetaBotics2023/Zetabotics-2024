package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeCommands.PickupFromGroundCommand;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.MirrablePose2d;

public class GoToPoseWhilePickingUp extends Command{
    
    private GoToPositionAfterTime goToPositionAfterTime;
    private PickupFromGroundCommand pickupFromGroundCommand;

    public GoToPoseWhilePickingUp(DriveSubsystem m_driveSubsystem, PickupFromGroundCommand pickupFromGroundCommand , MirrablePose2d pose, double time) {
        this.goToPositionAfterTime = new GoToPositionAfterTime(new GoToPositionAuton(m_driveSubsystem, pose), time);
        this.pickupFromGroundCommand = pickupFromGroundCommand;
    }

    public void initialize() {
        this.pickupFromGroundCommand.schedule();
        this.goToPositionAfterTime.schedule();
    }

    public void execute() {
        
    }

    public void end() {
        
    }

    public boolean isFinished() {
        return this.goToPositionAfterTime.isFinished() && this.pickupFromGroundCommand.isFinished();
    }
}
