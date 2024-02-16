package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/*
A simple command that will run another command when a supplier returns true.
*/
public class RunCommandUponConditionCommand extends Command {

    private Command command;
    private boolean commandHasRan = false;
    private Supplier<Boolean> conditionalSupplier;

    public RunCommandUponConditionCommand(Command command, Supplier<Boolean> conditionalSupplier) {
        this.command = command;
        this.conditionalSupplier = conditionalSupplier;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (this.conditionalSupplier.get()) {
            this.command.schedule();
            commandHasRan = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return commandHasRan && this.command.isFinished();
    }
}
