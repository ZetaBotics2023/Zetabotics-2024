package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;

public class GoToPositionAfterTime extends Command{
    private GoToPositionAuton goToPosition;
    private double waitTime;
    private WaitCommand waitCommand;
    public GoToPositionAfterTime(GoToPositionAuton goToPosition, double waitTime) {
        this.goToPosition = goToPosition;
        this.waitTime = waitTime;
    }

    public void initialize() {
    }

    public void execute() {
        if(this.waitCommand == null) {
            this.waitCommand = new WaitCommand(this.waitTime);
            this.waitCommand.schedule();
        } 
        if(this.waitCommand != null) {
            if(this.waitCommand.isFinished() && !this.goToPosition.isScheduled()) {
                this.goToPosition.schedule();
                this.waitCommand = null;
            }
        }
    }

    public void end() {
        this.waitCommand = null;
    }

    public boolean isFinished() {
        return this.goToPosition.isFinished();
    }
}
