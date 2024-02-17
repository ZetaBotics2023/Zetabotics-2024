package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;  


public class ButtonBoard extends XboxController {

    private int buttonPreset = 0;
    private ArrayList<HashMap<Button, Pair<Runnable, Runnable>>> presetList = new ArrayList<>();

    public ButtonBoard(int port) {
        super(port);
    }

    public JoystickButton getButton(Button button) {
        return new JoystickButton(this, button.value);
    }

    public void buttonOnTrue(Button button, int presetSlot, Runnable onTrue, Runnable onFalse) {

        JoystickButton joystickButton = getButton(button);

        if (presetList.size() <= presetSlot) {
            if (presetList.size() == presetSlot) {
                presetList.add(new HashMap<>());
            } else {
                throw new RuntimeException("You cannot write to slot '" + presetSlot + "' until you first write to all previous slots!");
            }
        }
        HashMap<Button, Pair<Runnable, Runnable>> targetMap = presetList.get(presetSlot);

        targetMap.put(button, new Pair<Runnable,Runnable>(onTrue, onFalse));

        Subsystem[] subsystems = {};

        if (onTrue != null) {
            joystickButton.onTrue(
                Commands.runOnce(() -> {
                    buttonCallback(
                    button,
                    true
                    );
                }, 
                subsystems)
                );
        }
        
        if (onFalse != null) {
            joystickButton.onFalse(
                Commands.runOnce(() -> {
                    buttonCallback(
                    button,
                    true
                    );
                }, 
                subsystems)
                );
        }
    }

    public int getPreset() {
        return this.buttonPreset;
    }

    public void buttonCallback(Button button, boolean onTrue) {
        Pair<Runnable, Runnable> command = presetList.get(buttonPreset).get(button);
        if (onTrue) {
            command.getFirst().run();
        }
        else {
            command.getSecond().run();
        }
    }
}
