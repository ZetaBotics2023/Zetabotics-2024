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

    public static void pollPOVButtons() {
        /* 
        // autoShootPositionCommand
        m_buttonBoard.bindToAxis(0, m_buttonBoard.getController()::getPOV, 90, this.autoShootPositionCommand, Commands.runOnce(this.autoShootPositionCommand::cancel));
        final JoystickButton shootNoteAutoPose = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kA.value);
        shootNoteAutoPose.onTrue(this.autoShootPositionCommand);
        shootNoteAutoPose.onFalse(Commands.runOnce(this.autoShootPositionCommand::cancel));   

        
    
        // climbUpDualCommand
        m_buttonBoard.bindToAxis(0, m_buttonBoard.getController()::getPOV, 180, this.climbUpDualCommand, Commands.runOnce(this.climbUpDualCommand::cancel));
        final JoystickButton moveClimbersUp = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kLeftBumper.value);
        moveClimbersUp.onTrue(this.climbUpDualCommand);
        moveClimbersUp.onFalse(Commands.runOnce(this.climbUpDualCommand::cancel));

        // climbDownDualCommand
        m_buttonBoard.bindToAxis(0, m_buttonBoard.getController()::getPOV, 0, this.climbDownDualCommand, Commands.runOnce(this.climbDownDualCommand::cancel));
        final JoystickButton moveClimbersDown = new JoystickButton(m_buttonBoardAlternative, XboxController.Button.kRightBumper.value);
        moveClimbersDown.onTrue(this.climbDownDualCommand);
        moveClimbersDown.onFalse(Commands.runOnce(this.climbDownDualCommand::cancel));
        */
    }
}
