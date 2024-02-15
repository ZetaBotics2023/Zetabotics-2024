package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RunCommandUponConditionCommand extends Command {

    private Command command;
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
        
    }

    @Override
    public void end(boolean interrupted) {
        this.command.schedule();
    }

    @Override
    public boolean isFinished() {
        return this.conditionalSupplier.get();
    }
}
