package frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/*
 * A subsystem that allows us to control our LED light strip
 */
public class LEDSubsystem extends SubsystemBase{

    /* 
     * An enum to represent common color values
     */
    public enum RGBColor {

        Red(new int[] {255, 0, 0}),
        Orange(new int[] {255, 127, 0}),
        Yellow(new int[] {255, 255, 0}),
        Green(new int[] {0, 255, 0}),
        Blue(new int[] {0, 0, 255}),
        Purple(new int[] {255, 0, 255});

        
        public final int[] color;

        RGBColor(int[] color) {
            this.color = color;
        } 
    }

    /*
     * An enum to represent common color patterns
     */
    public enum Pattern {
        Rainbow(new int[][]{
        RGBColor.Red.color,
        RGBColor.Orange.color,
        RGBColor.Yellow.color,
        RGBColor.Green.color,
        RGBColor.Blue.color,
        RGBColor.Purple.color
        }),
        Christmas(new int[][]{
        RGBColor.Red.color,
        RGBColor.Green.color
        });

        public final int[][] pattern;
        Pattern(int[][] pattern) {
            this.pattern = pattern;
        }
    }
    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;
    public int m_rainbowFirstPixelHue = 0;
    private boolean playRainbow = true;

    public LEDSubsystem() {
        m_led = new AddressableLED(LEDConstants.kLEDPWMPort);

        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
    }

    /*
     * Sets every light on the strip to the same color
     */
    public void setSolidColor(int[] color) {
        this.playRainbow = false;
        SmartDashboard.putNumber("LED Color", color[0]);
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            SmartDashboard.putNumber("LED Set", i);
            m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
         }
         
         m_led.setData(m_ledBuffer);
    }

    /*
     * Sets every light on the strip to follow a pattern
     */
    public void setPattern(int[][] pattern) {
        this.playRainbow = false;
        int colorIndex = 0;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            int[] color = pattern[colorIndex];
            m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
            colorIndex++;
            if (colorIndex >= pattern.length) {
                colorIndex = 0;
            }
         }
         
         m_led.setData(m_ledBuffer);
    }

    private void rainbow() {
        if(this.playRainbow) {
            // For every pixel
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final var hue = (m_rainbowFirstPixelHue  + (i * 180 / m_ledBuffer.getLength())) % 180;
                // Set the value
                m_ledBuffer.setHSV(i, hue, 255, 128);
            }
            // Increase by to make the rainbow "move"
            m_rainbowFirstPixelHue += 3;
            // Check bounds
            m_rainbowFirstPixelHue %= 180;
            }
      }

    public void stopRainbow() {
        this.playRainbow = false;
    }

    public void startRainbow() {
        this.playRainbow = true;
    }
}
