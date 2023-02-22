// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors.colorsensor;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** ColorSensorIO using a {@link ColorSensorV3} */
public class ColorSensorIOREV3 implements ColorSensorIO {
  private final ColorSensorV3 m_colorSensor;

  public ColorSensorIOREV3(I2C.Port kPort) {
    m_colorSensor = new ColorSensorV3(kPort);

    if (!m_colorSensor.isConnected()) {
      DriverStation.reportError(
          "Unable to Communicate with the Color Sensor, please make sure it is plugged into the correct I2C port and is powered.",
          false);
    }
  }

  /**
   * Get the color sensor object used by the IO.
   *
   * @return color sensor.
   */
  public ColorSensorV3 getColorSensor() {
    return m_colorSensor;
  }

  @Override
  public void updateInputs(ColorSensorIOInputs inputs) {
    inputs.InfraredValue = m_colorSensor.getIR();
    inputs.RedValue = m_colorSensor.getRed();
    inputs.BlueValue = m_colorSensor.getBlue();
    inputs.GreenValue = m_colorSensor.getGreen();
    inputs.ProximityValue = m_colorSensor.getProximity();
  }

  @Override
  public Color getColor() {
    return m_colorSensor.getColor();
  }

  @Override
  public Color8Bit getColor8Bit() {
    return new Color8Bit(getColor());
  }
}
