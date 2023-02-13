package frc.robot.sensors.encoder;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import org.talon540.hardware.CANDeviceConfig;

public class QuadratureEncoderIOCANCoder implements QuadratureEncoderIO {
  private final WPI_CANCoder m_encoder;

  public QuadratureEncoderIOCANCoder(CANDeviceConfig deviceConfig) {
    m_encoder = new WPI_CANCoder(deviceConfig.id, deviceConfig.controller);

    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";

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
