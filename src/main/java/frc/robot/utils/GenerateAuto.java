package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GenerateAuto {
    public static SequentialCommandGroup generateAuto(ArrayList<Command> autoCommands) {
        Command[] commands = new Command[autoCommands.size()];
        for(int i = 0; i < autoCommands.size(); i++) {
            commands[i] = autoCommands.get(i);
        }
        return new SequentialCommandGroup(commands);
    }
}
