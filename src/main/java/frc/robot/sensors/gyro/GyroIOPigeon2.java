package frc.robot.sensors.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

/** GyroIO using a {@link WPI_Pigeon2}. */
public class GyroIOPigeon2 implements GyroIO {
  private final WPI_Pigeon2 m_gyro;
  private final Pigeon2Accelerometer m_accelerometer;

  /** Create a GyroIO using a {@link WPI_Pigeon2}. */
  public GyroIOPigeon2(int id) {
    m_gyro = new WPI_Pigeon2(id);
    m_accelerometer = new Pigeon2Accelerometer(m_gyro);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.GyroYawRad = Math.toRadians(m_gyro.getYaw());
    inputs.GyroPitchRad = Math.toRadians(m_gyro.getPitch());
    inputs.GyroRollRad = Math.toRadians(m_gyro.getRoll());
    inputs.GyroRateRadPerSecond = Math.toRadians(m_gyro.getRate());

    inputs.AccelX = m_accelerometer.getX();
    inputs.AccelY = m_accelerometer.getY();
    inputs.AccelZ = m_accelerometer.getZ();
  }

  @Override
  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  @Override
  public void resetHeading() {
    m_gyro.reset();
  }

  /**
   * Get the Gyro object used by the IO.
   *
   * @return gyro.
   */
  public WPI_Pigeon2 getGyro() {
    return m_gyro;
  }
}
