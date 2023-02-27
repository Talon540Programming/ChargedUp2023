package frc.lib;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Class used to represent analog LED Strips plugged into 3 PWM ports representing the three color lanes. Each channel should have an N-Channel MOSFET wired to the gate pin.
 */
public class LEDStrips {
    private final PWM redChannel;
    private final PWM greenChannel;
    private final PWM blueChannel;

    /**
     * Create a LED Strip Helper Class used to control Analog LED Strips.
     *
     * @param redPin the PWM pin the Red Channel is plugged into.
     * @param greenPin the PWM pin the Green Channel is plugged into.
     * @param bluePin the PWM pin the Blue Channel is plugged into.
     */
    public LEDStrips(int redPin, int greenPin, int bluePin) {
        this.redChannel = new PWM(redPin);
        this.greenChannel = new PWM(greenPin);
        this.blueChannel = new PWM(bluePin);
    }

    /**
     * Set the value of the RED Channel on the LED Strips.
     *
     * @param value value [0, 255]
     */
    public void setRedValue(int value) {
        redChannel.setRaw(MathUtil.clamp(value, 0, 255));
    }

    /**
     * Set the value of the GREEN Channel on the LED Strips.
     *
     * @param value value [0, 255]
     */
    public void setGreenValue(int value) {
        greenChannel.setRaw(MathUtil.clamp(value, 0, 255));
    }

    /**
     * Set the value of the BLUE Channel on the LED Strips.
     *
     * @param value value [0, 255]
     */
    public void setBlueValue(int value) {
        blueChannel.setRaw(MathUtil.clamp(value, 0, 255));
    }

    /**
     * Set the color of the LED Strips from a {@link Color8Bit} object.
     *
     * @param color the color to set.
     */
    public void setColor(Color8Bit color) {
        setRedValue(color.red);
        setGreenValue(color.green);
        setBlueValue(color.blue);
    }

    /**
     * Set the color of the LED Strips from a {@link Color} object.
     *
     * @param color the color to set.
     */
    public void setColor(Color color) {
        setColor(new Color8Bit(color));
    }
}
