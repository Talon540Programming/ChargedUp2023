// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors.colorsensor;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
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

  default Color getColor() {
    return new Color(0, 0, 0);
  }

  default int getInfrared() {
    return 0;
  }

  default int getRed() {
    return 0;
  }

  default int getGreen() {
    return 0;
  }

  default int getBlue() {
    return 0;
  }

  default int getProximity() {
    return 0;
  }

  default boolean isAcceptableDistance() {
    return (getProximity() > Constants.Grabber.kMinimumAcceptableProximity);
  }
}
