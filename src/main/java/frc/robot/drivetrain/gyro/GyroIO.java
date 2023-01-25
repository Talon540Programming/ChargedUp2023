package frc.robot.drivetrain.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

/**
 * IO interfacing layer used to represent a gyroscope.
 */
public interface GyroIO extends LoggedIO<GyroIO.GyroIOInputs> {
  @AutoLog
  public static class GyroIOInputs {
    public double GyroYawRad = 0.0;
    public double GyroPitchRad = 0.0;
    public double GyroRollRad = 0.0;
    public double GyroRateRadPerSecond = 0.0;
  }

  @Override
  public default void updateInputs(GyroIOInputs inputs) {}

  /**
   * Get the heading of the robot as a {@link Rotation2d} object.
   *
   * @return robot heading.
   */
  public default Rotation2d getRotation2d() {
    return new Rotation2d();
  }

  /**
   * Get the Yaw (rotation around the z-axis) of the gyro in radians.
   *
   * @return gyro yaw.
   */
  public default double getYaw() {
    return 0;
  }

  /**
   * Get the Pitch (rotation around the y-axis) of the gyro in radians.
   *
   * @return gyro pitch.
   */
  public default double getPitch() {
    return 0;
  }

  /**
   * Get the Roll (rotation around the x-axis) of the gyro in radians.
   *
   * @return gyro roll.
   */
  public default double getRoll() {
    return 0;
  }

  /**
   * Reset the yaw of the gyro to 0.
   */
  public default void resetHeading() {}

  /**
   * Check if the Gyroscope is at a level plane.
   *
   * @return whether the gyroscope is level.
   */
  public default boolean isLevel() {
    return Math.abs(getPitch())
        < Math.toRadians(Constants.Drivetrain.kRobotStabilizationToleranceDegrees);
  }

  /**
   * Represent the position of the gyroscope as a {@link Rotation3d} object. Used to supplement the weak WPI interface.
   *
   * @return gyro data as a Rotation3d.
   */
  public default Rotation3d getRotation3d() {
    return new Rotation3d(getRoll(), getPitch(), getYaw());
  }
}
