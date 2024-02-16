package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonConfigurationConstants;
import frc.robot.commands.SequentialGroupCommand;

public class GenerateAuto {
    public static SequentialGroupCommand generateAuto(String autonName, ArrayList<Command> autoCommands) {
        Command[] commands = new Command[autoCommands.size()];
        for(int i = 0; i < autoCommands.size(); i++) {
            commands[i] = autoCommands.get(i);
        }
        
        return new SequentialGroupCommand(commands);
    }
}
