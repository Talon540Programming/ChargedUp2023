package frc.robot.drivetrain.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareDevices;

/**
 * GyroIO using a {@link WPI_Pigeon2}.
 */
public class GyroIOPigeon implements GyroIO {
  private final WPI_Pigeon2 m_gyro;

  /**
   * Create a GyroIO using a {@link WPI_Pigeon2}.
   */
  public GyroIOPigeon() {
    switch (Constants.kCurrentMode) {
      case PROTO -> {
        m_gyro =
            new WPI_Pigeon2(
                HardwareDevices.PROTO.kRobotGyroConfig.id,
                HardwareDevices.PROTO.kRobotGyroConfig.controller);
      }
      case COMP -> {
        m_gyro =
            new WPI_Pigeon2(
                HardwareDevices.COMP.kRobotGyroConfig.id,
                HardwareDevices.COMP.kRobotGyroConfig.controller);
      }
      default -> throw new RuntimeException(
          "Shouldn't be using this IO system if running on a SIM robot");
    }
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
    inputs.GyroYawRad = Math.toRadians(m_gyro.getYaw());
    inputs.GyroPitchRad = Math.toRadians(m_gyro.getPitch());
    inputs.GyroRollRad = Math.toRadians(m_gyro.getRoll());
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
}
