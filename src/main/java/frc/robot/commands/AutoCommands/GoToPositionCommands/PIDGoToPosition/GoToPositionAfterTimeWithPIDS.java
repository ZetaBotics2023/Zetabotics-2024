package frc.robot.commands.AutoCommands.GoToPositionCommands.PIDGoToPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GoToPositionAfterTimeWithPIDS extends Command{
    private GoToPoseitionWithPIDSAuto goToPosition;
    private double waitTime;
    private WaitCommand waitCommand;
    public GoToPositionAfterTimeWithPIDS(GoToPoseitionWithPIDSAuto goToPosition, double waitTime) {
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
