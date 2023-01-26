package frc.robot.drivetrain.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import org.talon540.hardware.CANDeviceConfig;

/** GyroIO using a {@link WPI_Pigeon2}. */
public class GyroIOPigeon2 implements GyroIO {
  private final WPI_Pigeon2 m_gyro;

  /** Create a GyroIO using a {@link WPI_Pigeon2}. */
  public GyroIOPigeon2(GyroIOPigeon2Config config) {
    m_gyro =
            new WPI_Pigeon2(
                    config.gyro.id,
                    config.gyro.controller);

  }

  /**
   * Get the Gyro object used by the IO.
   *
   * @return gyro.
   */
  public WPI_Pigeon2 getGyro() {
    return m_gyro;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.GyroYawRad = getYaw();
    inputs.GyroPitchRad = getPitch();
    inputs.GyroRollRad = getRoll();
    inputs.GyroRateRadPerSecond = Math.toRadians(m_gyro.getRate());
  }

  @Override
  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  @Override
  public double getYaw() {
    return Math.toRadians(m_gyro.getYaw());
  }

  @Override
  public double getPitch() {
    return Math.toRadians(m_gyro.getPitch());
  }

  @Override
  public double getRoll() {
    return Math.toRadians(m_gyro.getRoll());
  }

  @Override
  public void resetHeading() {
    m_gyro.reset();
  }

  public record GyroIOPigeon2Config(CANDeviceConfig gyro) {}
}
