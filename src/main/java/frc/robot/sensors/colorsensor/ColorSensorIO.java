// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors.colorsensor;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.logging.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

/** IO interfacing layer used to represent a ColorSensor. */
public interface ColorSensorIO extends LoggedIO<ColorSensorIO.ColorSensorIOInputs> {
  @AutoLog
  class ColorSensorIOInputs {
    public long InfraredValue;
    public long RedValue;
    public long GreenValue;
    public long BlueValue;
    public long ProximityValue;
  }

  @Override
  default void updateInputs(ColorSensorIOInputs inputs) {}

  /**
   * Get the current Color being seen by the Color sensor as a {@link Color} object.
   *
   * @return current color.
   */
  default Color getColor() {
    return null;
  }

  /**
   * Get the current Color being seen by the Color sensor as a {@link Color8Bit} object.
   *
   * @return current color.
   */
  default Color8Bit getColor8Bit() {
    return null;
  }
}
