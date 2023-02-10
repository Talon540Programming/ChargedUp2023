// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors.colorsensor;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/** ColorSensorIO using a {@link ColorSensorV3} */
public class ColorSensorIOREV3 implements ColorSensorIO {
  private final ColorSensorV3 m_colorSensor;

  public ColorSensorIOREV3(I2C.Port kI2cPort) {
    m_colorSensor = new ColorSensorV3(kI2cPort);
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
    inputs.InfraredValue = getInfrared();
    inputs.RedValue = getRed();
    inputs.BlueValue = getBlue();
    inputs.GreenValue = getGreen();
    inputs.ProximityValue = getProximity();
  }

  public Color getColor() {
    return m_colorSensor.getColor();
  }

  @Override
  public int getInfrared() {
    return m_colorSensor.getIR();
  }

  @Override
  public int getRed() {
    return m_colorSensor.getRed();
  }

  @Override
  public int getGreen() {
    return m_colorSensor.getBlue();
  }

  @Override
  public int getBlue() {
    return m_colorSensor.getBlue();
  }

  @Override
  public int getProximity() {
    return m_colorSensor.getProximity();
  }
}
