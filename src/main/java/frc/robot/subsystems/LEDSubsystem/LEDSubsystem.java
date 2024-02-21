package frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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

    public LEDSubsystem() {
        m_led = new AddressableLED(LEDConstants.kLEDPWMPort);

        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /*
     * Sets every light on the strip to the same color
     */
    public void setSolidColor(int[] color) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
         }
         
         m_led.setData(m_ledBuffer);
    }

    /*
     * Sets every light on the strip to follow a pattern
     */
    public void setPattern(int[][] pattern) {
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
}
