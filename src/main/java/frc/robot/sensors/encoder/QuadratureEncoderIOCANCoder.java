package frc.robot.sensors.encoder;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class QuadratureEncoderIOCANCoder implements QuadratureEncoderIO {
  private final WPI_CANCoder m_encoder;

  public QuadratureEncoderIOCANCoder(int id, double absoluteOffsetDegrees) {
    m_encoder = new WPI_CANCoder(id);

    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.magnetOffsetDegrees = absoluteOffsetDegrees;

    m_encoder.configAllSettings(config);
  }

  @Override
  public void updateInputs(QuadratureEncoderIOInputs inputs) {
    inputs.AbsolutePositionRad = m_encoder.getAbsolutePosition();
    inputs.PositionRad = m_encoder.getPosition();
    inputs.VelocityRadPerSecond = m_encoder.getVelocity();
  }

  @Override
  public void setPosition(double position) {
    m_encoder.setPosition(position);
  }

  @Override
  public void resetToAbsolute() {
    m_encoder.setPositionToAbsolute();
  }
}
