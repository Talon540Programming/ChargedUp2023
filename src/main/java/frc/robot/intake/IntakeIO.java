package frc.robot.intake;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends LoggedIO<IntakeIO.IntakeInputs> {
    @AutoLog
    public class IntakeInputs {
        public double[] CurrentAmps = new double[] {};
        public double[] TemperatureCelsius = new double[] {};

        public double LeftVelocityRadPerSecond;
        public double RightVelocityRadPerSecond;

        public long InfraredValue;
        public long RedValue;
        public long GreenValue;
        public long BlueValue;
        public double ProximityValueCm;
    }

    default void setVoltage(double voltage) {}

    default Color8Bit getColor8Bit() { return null; }

    default void setNeutralMode(Constants.NeutralMode mode) {}
}
