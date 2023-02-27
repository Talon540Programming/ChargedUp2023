package frc.lib;


import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDStrips {
    private final PWM redChannel;
    private final PWM greenChannel;
    private final PWM blueChannel;

    public LEDStrips(int redPin, int greenPin, int bluePin) {
        this.redChannel = new PWM(redPin);
        this.greenChannel = new PWM(greenPin);
        this.blueChannel = new PWM(bluePin);
    }

    public void setRedValue(int value) {
        redChannel.setRaw(MathUtil.clamp(value, 0, 255));
    }

    public void setGreenValue(int value) {
        greenChannel.setRaw(MathUtil.clamp(value, 0, 255));
    }

    public void setBlueValue(int value) {
        blueChannel.setRaw(MathUtil.clamp(value, 0, 255));
    }

    public void setColor(Color8Bit color) {
        setRedValue(color.red);
        setGreenValue(color.green);
        setBlueValue(color.blue);
    }

    public void setColor(Color color) {
        setColor(new Color8Bit(color));
    }
}
