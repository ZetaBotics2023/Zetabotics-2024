package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GenerateAuto {
    public static SequentialCommandGroup generateAuto(ArrayList<Command> autoCommands) {
        return new SequentialCommandGroup((Command[]) autoCommands.toArray());
    }
}
