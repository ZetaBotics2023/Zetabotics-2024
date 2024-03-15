package frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

public class CTRELEDSubsystem extends SubsystemBase {

    private CANdle candle;
    private CANdleConfiguration config;

    public CTRELEDSubsystem() {
        candle = new CANdle(LEDConstants.kLEDCANID);
        this.config = new CANdleConfiguration();
        this.config.brightnessScalar = 1;
        this.config.statusLedOffWhenActive = true;
        this.config.v5Enabled = false;
        this.config.stripType = LEDStripType.RGB; 
        SmartDashboard.putBoolean("Set LED Color", true);
       
    }

    @Override
    public void periodic() {
    }

    public void setSolidColor(int[] rgb) {
        SmartDashboard.putBoolean("Set LED Color", true);
        candle.setLEDs(rgb[0], rgb[1], rgb[2]);
    }

    public void animate(Animation animation) {
        candle.animate(new StrobeAnimation(0, 255, 0, 155, .5, 128));//new Animation() {
            ;//FireAnimation(1, .5, 155, 128, 1));
    }
}