package frc.robot.sensors.encoder;

import frc.lib.logging.LoggedIO;

public interface QuadratureEncoderIO
    extends LoggedIO<QuadratureEncoderIO.QuadratureEncoderIOInputs> {
  class QuadratureEncoderIOInputs {
    public double AbsolutePositionRad;
    public double PositionRad;
    public double VelocityRadPerSecond;
  }

  @Override
  default void updateInputs(QuadratureEncoderIOInputs inputs) {}

  default void setPosition(double position) {}

  default void resetToAbsolute() {}
}
