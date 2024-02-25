package frc.robot.commands.AutoCommands;

import com.ctre.phoenix6.controls.CoastOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;

public class WaitCommandWrapper extends Command{
    private double waitTime;
    private WaitCommand waitCommand = null;
    public WaitCommandWrapper(double waitTime) {
        this.waitTime = waitTime;
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        if(this.waitCommand == null) {
            this.waitCommand = new WaitCommand(waitTime);
            this.waitCommand.schedule();
        } 
        if(this.waitCommand != null) {
            return this.waitCommand.isFinished();
        }

        return false;
    }

}
