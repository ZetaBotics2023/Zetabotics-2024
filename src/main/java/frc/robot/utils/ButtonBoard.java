package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands.AutoShootCommands.AutoShootPositionCenterCommand;
import frc.robot.commands.AutoCommands.AutoShootCommands.AutoShootPositionLeftCommand;
import frc.robot.commands.AutoCommands.AutoShootCommands.AutoShootPositionRightCommand;  

public class ButtonBoard {

    public enum Button {
        /** Left bumper. */
        kBottomRightBlack(5),
        /** Right bumper. */
        kTopRightBlack(6),
        /** Left stick. */
        kWhiteLeft(9),
        /** Right stick. */
        kWhiteRight(10),
        /** A. */
        kGreen(1),
        /** B. */
        kRed(2),
        /** X. */
        kBlue(3),
        /** Y. */
        kYellow(4),
        /** Back. */
        kBlackSelect(7),
        /** Start. */
        kBlackStart(8);
    
        /** Button value. */
        public final int value;
        public boolean previousValue;
    
        Button(int value) {
          this.value = value;
        }
    }


    private int buttonPreset = 0;
    private XboxController controller;

    public ButtonBoard(int port) {
        this.controller = new XboxController(port);
    }

    public void bindToButton(int slot, Button button, Command onTrue, Command onFalse) {
        JoystickButton joystickButton = new JoystickButton(this.controller, button.value);
        BooleanSupplier slotBoolSupplier = () -> {return slot == buttonPreset;};
        joystickButton.and(slotBoolSupplier);
        joystickButton.onFalse(onFalse);
        joystickButton.onTrue(onTrue);
    }

    public int getPreset() {
        return this.buttonPreset;
    }
    public void setPreset(int preset) {
        this.buttonPreset = preset;
    }

    public XboxController getController() {
        return controller;
    }

    public static void pollPOVButtons(ButtonBoard buttonBoard, Command... commands) {
        int pov = buttonBoard.getController().getPOV();

        if (pov == 270 && !commands[0].isScheduled()) {
            commands[0].schedule();
        } else if(commands[0].isScheduled()) {
            commands[0].cancel();
        }

        if (pov == 0 && !commands[1].isScheduled()) {
            commands[1].schedule();
        } else if(commands[1].isScheduled()){
            commands[1].cancel();
        }

        if (pov == 90 && !commands[2].isScheduled() ) {
            commands[2].schedule();
        } else if(commands[2].isScheduled()){
            commands[2].cancel();
        }
    }
}
