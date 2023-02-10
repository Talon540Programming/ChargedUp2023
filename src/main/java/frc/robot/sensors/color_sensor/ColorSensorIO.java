// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors.color_sensor;

import edu.wpi.first.wpilibj.util.Color;
import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

/** IO interfacing layer used to represent a gyroscope. */
public interface ColorSensorIO extends LoggedIO<ColorSensorIO.ColorSensorIOInputs> {
  @AutoLog
  class ColorSensorIOInputs {
    public Color normalizedColor;
    public int infrared;
    public int redValue;
    public int greenValue;
    public int blueValue;
    public int proximity;
    public boolean acceptableDistance;
    public String gamePiece;
  }

  @Override
  default void updateInputs(ColorSensorIOInputs inputs) {}

  /**
   * Get the normalized color value (average of RGB) as a {@link Color} object.
   *
   * @return Normalized color value as {@link Color} object.
   */
  default Color getNormalizedColor() {
    return new Color(0, 0, 0);
  }

  /**
   * Get the infrared measurement of the color sensor as an int.
   *
   * @return infrared intensity/
   */
  default int getInfrared() {
    return 0;
  }

  /**
   * Get the red light component of the color detected as an int.
   *
   * @return red light component
   */
  default int getRed() {
    return 0;
  }

  /**
   * Get the green light component of the color detected as an int.
   *
   * @return green light component
   */
  default int getGreen() {
    return 0;
  }

  /**
   * Get the blue light component of the color detected as an int.
   *
   * @return blue light component
   */
  default int getBlue() {
    return 0;
  }

  /**
   * Get the estimated proximity of the color detected as an integer value.
   *
   * @return relative proximity
   */
  default int getProximity() {
    return 0;
  }

  /**
   * Check if the distance is close enough for an accurate color reading.
   *
   * @return if the distance is close enough for accurate reading.
   */
  default boolean isAcceptableDistance() {
    return (getProximity() > Constants.Grabber.kMinimumAcceptableProximity);
  }

  default String getGamePiece() {
    if (isAcceptableDistance()) {
      if (getNormalizedColor().equals(Color.kYellow))
        return "Cone"; // TODO: Color needs tuning to make sure it's the right one. May not be
      // standard
      else if (getNormalizedColor().equals(Color.kMediumPurple))
        return "Cube"; // TODO: Color needs tuning to make sure it's the right one. May not be
      // standard
    }
    return "No Game Piece";
  }
}
