package frc.robot.sensors.encoder;

import frc.lib.logging.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

/** IO abstraction for interfacing with a Quadrature encoder. */
public interface QuadratureEncoderIO
    extends LoggedIO<QuadratureEncoderIO.QuadratureEncoderIOInputs> {
  @AutoLog
  class QuadratureEncoderIOInputs {
    public double AbsolutePositionRad;
    public double PositionRad;
    public double VelocityRadPerSecond;
  }

  @Override
  default void updateInputs(QuadratureEncoderIOInputs inputs) {}

  /**
   * Set the position of the encoder.
   *
   * @param position position to set.
   */
  default void setPosition(double position) {}

  /** Reset the position of the encoder to the absolute position. */
  default void resetToAbsolute() {}
}
