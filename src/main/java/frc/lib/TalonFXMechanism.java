package frc.lib;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.math.util.Units;

public class TalonFXMechanism {
  public static final double IntegratedEncoderResolution = 2048.0;

  private final TalonFXSensorCollection m_collection;
  private final double m_gearing;
  private final double m_radius;
  private boolean m_inverted;

  public TalonFXMechanism(
      TalonFXSensorCollection collection, double gearing, double radius, boolean inverted) {
    m_collection = collection;
    m_gearing = gearing;
    m_radius = radius;
    m_inverted = inverted;
  }

  public void setInverted(boolean inverted) {
    m_inverted = inverted;
  }

  public double getVelocityRadPerSecond() {
    return Units.rotationsPerMinuteToRadiansPerSecond(getMotorRPM());
  }

  public double getVelocityMetersPerSecond() {
    return getVelocityRadPerSecond() * m_radius;
  }

  public double getPositionRadians() {
    return Units.rotationsToRadians(getMotorRotations()) / m_gearing;
  }

  public double getPositionMeters() {
    return getPositionRadians() * m_radius;
  }

  public void resetPosition(double positionTalonFXTicks) {
    m_collection.setIntegratedSensorPosition(positionTalonFXTicks, 0);
  }

  public double getMotorRotations() {
    return getRawPosition() / IntegratedEncoderResolution;
  }

  public double getMotorRPM() {
    return getRawVelocity() * 600 / IntegratedEncoderResolution;
  }

  /**
   * Return the raw velocity of the motor in CTRE encoder ticks / 100ms.
   *
   * @return velocity of the integrated encoder in CTRE units.
   */
  public double getRawVelocity() {
    return m_collection.getIntegratedSensorVelocity() * (m_inverted ? -1 : 1);
  }

  /**
   * Return the position of the encoder
   *
   * @return integrated encoder position.
   */
  public double getRawPosition() {
    return m_collection.getIntegratedSensorPosition() * (m_inverted ? -1 : 1);
  }

  /**
   * Return the absolute position of the encoder.
   *
   * @return integrated encoder absolute position.
   */
  public double getRawAbsolutePosition() {
    return m_collection.getIntegratedSensorAbsolutePosition();
  }

  public TalonFXSensorCollection getCollection() {
    return m_collection;
  }
}
