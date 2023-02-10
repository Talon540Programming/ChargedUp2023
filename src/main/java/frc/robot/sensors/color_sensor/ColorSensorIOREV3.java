// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors.color_sensor;

import com.revrobotics.ColorSensorV3;
import frc.robot.constants.HardwareDevices;
import edu.wpi.first.wpilibj.I2C;

/** ColorSensorIO using a {@link ColorSensorV3} */
public class ColorSensorIOREV3 implements ColorSensorIO {
    private final ColorSensorV3 m_colorSensor;

    /** ColorSensorIO using a {@link ColorSensorV3} */
    public ColorSensorIOREV3(I2C.Port i2cPort) {
        m_colorSensor = new ColorSensorV3(i2cPort);
    }

    /**
     * Get the color sensor object used by the IO
     * @return color sensor.
     */
    public ColorSensorV3 getColorSensor() {
        return m_colorSensor;
    }

    @Override
    public void updateInputs(ColorSensorIOInputs inputs) {
        inputs.normalizedColor = getNormalizedColor();
        inputs.infrared = getInfrared();
        inputs.redValue = getRed();
        inputs.blueValue = getBlue();
        inputs.greenValue = getGreen();
        inputs.proximity = getProximity();
        inputs.acceptableDistance = isAcceptableDistance();
        inputs.gamePiece = getGamePiece();
    }

    
}
