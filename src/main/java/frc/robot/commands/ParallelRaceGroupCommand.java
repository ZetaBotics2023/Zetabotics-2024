package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ParallelRaceGroupCommand extends Command {
    Command commandOne;
    Command commandTwo;
    public ParallelRaceGroupCommand(Command commandOne, Command commandTwo) {
        this.commandOne = commandOne;
        this.commandTwo = commandTwo;
    }

    @Override
    public void initialize() {
        this.commandOne.schedule();
        this.commandTwo.schedule();
        SmartDashboard.putBoolean("Race Command", false);

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Race Command", true);
        if (this.commandOne.isFinished()) this.commandTwo.cancel();
        if (this.commandTwo.isFinished()) this.commandOne.cancel();

    }

    @Override
    public boolean isFinished() {
        return this.commandOne.isFinished() || this.commandTwo.isFinished();
    }
}
