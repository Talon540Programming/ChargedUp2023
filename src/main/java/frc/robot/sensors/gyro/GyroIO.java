package frc.robot.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.logging.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

/** IO interfacing layer used to represent a gyroscope. */
public interface GyroIO extends LoggedIO<GyroIO.GyroIOInputs> {
  @AutoLog
  class GyroIOInputs {
    public double GyroYawRad;
    public double GyroPitchRad;
    public double GyroRollRad;
    public double GyroRateRadPerSecond;
  }

  @Override
  default void updateInputs(GyroIOInputs inputs) {}

  /**
   * Get the heading of the robot as a {@link Rotation2d} object.
   *
   * @return robot heading.
   */
  default Rotation2d getRotation2d() {
    return new Rotation2d();
  }

  /** Reset the yaw of the gyro to 0. */
  default void resetHeading() {}
}
