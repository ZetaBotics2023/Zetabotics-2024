package frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

public class CTRELEDSubsystem extends SubsystemBase {

    private CANdle candle;

    public CTRELEDSubsystem() {
        candle = new CANdle(LEDConstants.kLEDCANID);
    }

    public void setSolidColor(int[] rgb) {
        candle.setLEDs(rgb[0], rgb[1], rgb[2]);
    }

    public void animate(Animation animation) {
        candle.animate(animation);
    }
}
