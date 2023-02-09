package frc.robot.sensors.encoder;

import frc.lib.logging.LoggedIO;

public interface QuadratureEncoderIO extends LoggedIO<QuadratureEncoderIO.QuadratureEncoderIOInputs> {
    public class QuadratureEncoderIOInputs {
        public double AbsolutePositionRad;
        public double VelocityRadPerSecond;
    }

    @Override
    default void updateInputs(QuadratureEncoderIOInputs inputs) {};
}
