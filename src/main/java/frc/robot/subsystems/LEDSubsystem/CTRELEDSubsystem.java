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
       this.configuration = new CANdleConfiguration();
        this.configuration.statusLedOffWhenActive = false;
        this.configuration.disableWhenLOS = false;
        this.configuration.stripType = LEDStripType.RGB;
        this.configuration.vBatOutputMode = VBatOutputMode.Modulated;
        this.candle.configAllSettings(configuration);
       
    }

    @Override
    public void periodic() {
        
    }

    public void setSolidColor(int[] rgb) {
        this.candle.setLEDs(rgb[0], rgb[1], rgb[2]);
        SmartDashboard.putBoolean("Set LED Color", true);
        candle.setLEDs(rgb[0], rgb[1], rgb[2]);
    }

    public void animate(Animation animation) {
        candle.animate(new StrobeAnimation(0, 255, 0, 155, 1, 128));//new Animation() {
            //FireAnimation(1, .5, 155, 128, 1));
        this.candle.animate(animation);
    }

    public void TwinkleAnimation(int[] rgbw) { 
        this.candle.animate(new TwinkleAnimation(rgbw[0], rgbw[1], rgbw[2], rgbw[3], .98, this.LEDCount, TwinklePercent.Percent100));
    }

    public void FireAnimation() { 
        this.candle.animate(new FireAnimation(1, .5, this.LEDCount, .5, .5));
    }

    public void larsonAnimation(int[] rgbw) {
        this.candle.animate(new LarsonAnimation(rgbw[0], rgbw[1], rgbw[2], rgbw[3], .5, this.LEDCount, BounceMode.Center, 7));
    }

    public void rainbowAnimation() {
        this.candle.animate(new RainbowAnimation(1, 1, this.LEDCount));
    }

}