package frc.robot.sensors.encoder;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.lib.logging.LoggedIO;

public interface QuadratureEncoderIO
    extends LoggedIO<QuadratureEncoderIO.QuadratureEncoderIOInputs> {
  public class QuadratureEncoderIOInputs {
    public double AbsolutePositionRad;
    public double PositionRad;
    public double VelocityRadPerSecond;
  }

  @Override
  default void updateInputs(QuadratureEncoderIOInputs inputs) {}

  default public void setPosition(double position) {}

  default public void resetToAbsolute() {}
}
