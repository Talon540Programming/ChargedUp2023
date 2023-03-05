package frc.robot.intake.wrist;

import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeWristIO extends LoggedIO<IntakeWristIO.IntakeWristInputs> {
  @AutoLog
  class IntakeWristInputs {
    public double VelocityRadPerSecond;
    public double CurrentAmps;
    public double TemperatureCelsius;
  }

  @Override
  default void updateInputs(IntakeWristInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void setNeutralMode(Constants.NeutralMode mode) {}
}
