package frc.robot.sensors.encoder;

import frc.lib.logging.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

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

  default void setPosition(double position) {}

  default void resetToAbsolute() {}
}
