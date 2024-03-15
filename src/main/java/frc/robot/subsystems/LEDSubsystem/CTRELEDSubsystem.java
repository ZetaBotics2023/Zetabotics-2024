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
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class CTRELEDSubsystem extends SubsystemBase {

    private CANdle candle;
    private CANdleConfiguration configuration;
    private int LEDCount = 128;
    
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
        this.configuration = new CANdleConfiguration();
        this.configuration.statusLedOffWhenActive = false;
        this.configuration.disableWhenLOS = false;
        this.configuration.stripType = LEDStripType.RGB;
        this.configuration.vBatOutputMode = VBatOutputMode.Modulated;
        this.candle.configAllSettings(configuration);
    }

    public void setSolidColor(int[] rgb) {
        this.candle.setLEDs(rgb[0], rgb[1], rgb[2]);
        SmartDashboard.putBoolean("Set LED Color", true);
        candle.setLEDs(rgb[0], rgb[1], rgb[2]);
    }

    public void animate(Animation animation) {
        candle.animate(new StrobeAnimation(0, 255, 0, 155, .5, 128));//new Animation() {
            ;//FireAnimation(1, .5, 155, 128, 1));
        this.candle.animate(animation);
    }
}
