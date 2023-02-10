// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors.color_sensor;

import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

/** IO interfacing layer used to represent a gyroscope. */
public interface ColorSensorIO extends LoggedIO<ColorSensorIO.ColorSensorIOInputs> {
  @AutoLog
  class ColorSensorIOInputs {
    public double GyroYawRad;
    public double GyroPitchRad;
    public double GyroRollRad;
    public double GyroRateRadPerSecond;
  }

  @Override
  default void updateInputs(ColorSensorIOInputs inputs) {}

  /**
   * Get the Yaw (rotation around the z-axis) of the gyro in radians.
   *
   * @return gyro yaw.
   */
  default double getYaw() {
    return 0;
  }

  /**
   * Get the Pitch (rotation around the y-axis) of the gyro in radians.
   *
   * @return gyro pitch.
   */
  default double getPitch() {
    return 0;
  }

  /**
   * Get the Roll (rotation around the x-axis) of the gyro in radians.
   *
   * @return gyro roll.
   */
  default double getRoll() {
    return 0;
  }

  /** Reset the yaw of the gyro to 0. */
  default void resetHeading() {}

  /**
   * Check if the Gyroscope is at a level plane.
   *
   * @return whether the gyroscope is level.
   */
  default boolean isLevel() {
    return Math.abs(getPitch())
        < Math.toRadians(Constants.Drivetrain.kRobotStabilizationToleranceDegrees);
  }

  /**
   * Represent the position of the gyroscope as a {@link Rotation3d} object. Used to supplement the
   * weak WPI interface.
   *
   * @return gyro data as a Rotation3d.
   */
}
