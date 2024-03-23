package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class EndUponConditionCommand extends Command{
    private Supplier<Boolean> condition;
    public EndUponConditionCommand(Supplier<Boolean> condition) {
        this.condition = condition;
    }

    
    public boolean isFinished() {
        return this.condition.get();
    }
}
