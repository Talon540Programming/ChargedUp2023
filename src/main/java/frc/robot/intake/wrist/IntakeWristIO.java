package frc.robot.intake.wrist;

import frc.lib.logging.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeWristIO extends LoggedIO<IntakeWristIO.IntakeWristInputs> {
  @AutoLog
  public class IntakeWristInputs {
    public double VelocityRadPerSecond;
    public double CurrentAmps;
    public double TemperatureCelsius;
  }

  @Override
  default void updateInputs(IntakeWristInputs inputs) {}

  default void setVoltage(double voltage) {}
}
