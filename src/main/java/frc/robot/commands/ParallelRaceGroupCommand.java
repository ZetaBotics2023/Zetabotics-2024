package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

/*
A simple command to replace the WPILib ParallelRaceCommandGroup that doesn't work.
It only works with two commands as of right now. 
*/
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
        //SmartDashBoard.putBoolean("Race Command", false);

    }

    @Override
    public void end(boolean interrupted) {
        //SmartDashBoard.putBoolean("Race Command", true);
        // If one is finished, end the other
        if (this.commandOne.isFinished()) this.commandTwo.cancel();
        if (this.commandTwo.isFinished()) this.commandOne.cancel();

    }

    @Override
    public boolean isFinished() {
        return this.commandOne.isFinished() || this.commandTwo.isFinished();
    }
}
