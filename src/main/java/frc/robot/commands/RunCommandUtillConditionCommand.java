package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/*
A simple command that will run another command when a supplier returns true.
*/
public class RunCommandUtillConditionCommand extends Command {

    private Command command;
    private boolean commandHasRan = false;
    private Supplier<Boolean> conditionalSupplier;

    public RunCommandUtillConditionCommand(Command command, Supplier<Boolean> conditionalSupplier) {
        this.command = command;
        this.conditionalSupplier = conditionalSupplier;
    }

    @Override
    public void initialize() {
        this.command.schedule();
    }

    @Override
    public void execute() {
        if (this.command.isScheduled() && this.conditionalSupplier.get()) {
            this.command.cancel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("ENDED Command", true);
    }

    @Override
    public boolean isFinished() {
        return !this.command.isScheduled();
    }
}
