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
import edu.wpi.first.wpilibj2.command.button.Trigger;  

public class ButtonBoard extends XboxController {

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
    
    public enum AlternateButton {
        /** Left X. */
        kRedUp(0, 1),
        kRedDown(0, -1),
        /** Left Y. */
        kRedLeft(1, -1),
        kRedRight(1, 1),
        /** Left trigger. */
        kBlackBottomLeft(2, 1),
        /** Right trigger. */
        kBlackTopLeft(3, 1);
    
        /** Axis value. */
        public final int value;
        public boolean previousValue;
        public final int triggerValue;
        
        /*
         * Construct an alternate button that converts an axis into a button press.
         */
        AlternateButton(int value, int triggerValue) {
          this.value = value;
          this.triggerValue = triggerValue;
        }
    }

    private int buttonPreset = 0;

    public ButtonBoard(int port) {
        super(port);
    }

    public void createButtonTrigger(ButtonBoard.Button button, int presetSlot, Runnable onTrue, Runnable onFalse) {
        Trigger onTrueTrigger = new Trigger(
            () -> {
                
                if (getPreset() != presetSlot) return false;

                // Indexes start at one for getRawButtonPressed, so add one. //TODO: Verify if this is correct
                boolean isPressed = getRawButtonPressed(button.value);
                boolean wasPressed = button.previousValue;
                if (isPressed && !wasPressed) {
                    button.previousValue = true;
                    return true;
                }
                return false;
            }
        );

        Trigger onFalseTrigger = new Trigger(
            () -> {

                if (getPreset() != presetSlot) return false;

                boolean isPressed = getRawButtonPressed(button.value);
                boolean wasPressed = button.previousValue;
                if (!isPressed && wasPressed) {
                    button.previousValue = false;
                    return true;
                }
                return false;
            }
        );

        onTrueTrigger.onTrue(Commands.runOnce(onTrue));
        onFalseTrigger.onTrue(Commands.runOnce(onFalse));

        
    }

    public void createButtonTrigger(ButtonBoard.AlternateButton button, int presetSlot, Runnable onTrue, Runnable onFalse) {
        Trigger onTrueTrigger = new Trigger(
            () -> {

                if (getPreset() != presetSlot) return false;

                // Indexes start at zero for getRawAxis, so DON'T add one. //TODO: Verify if this is correct
                boolean isPressed = getRawAxis(button.value) == button.triggerValue;
                boolean wasPressed = button.previousValue;
                if (isPressed && !wasPressed) {
                    button.previousValue = true;
                    return true;
                }
                return false;
            }
        );

        Trigger onFalseTrigger = new Trigger(
            () -> {

                if (getPreset() != presetSlot) return false;

                boolean isPressed = getRawAxis(button.value) == button.triggerValue;
                boolean wasPressed = button.previousValue;
                if (!isPressed && wasPressed) {
                    button.previousValue = false;
                    return true;
                }
                return false;
            }
        );

        onTrueTrigger.onTrue(Commands.runOnce(onTrue));
        onFalseTrigger.onTrue(Commands.runOnce(onFalse));

        
    }

    public int getPreset() {
        return this.buttonPreset;
    }
    public void setPreset(int preset) {
        this.buttonPreset = preset;
    }
}
