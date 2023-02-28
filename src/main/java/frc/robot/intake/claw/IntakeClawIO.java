package frc.robot.intake.claw;

import frc.lib.logging.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeClawIO extends LoggedIO<IntakeClawIO.IntakeClawInputs> {
  @AutoLog
  public class IntakeClawInputs {
    public double CurrentAmps;
    public double TemperatureCelsius;
  }

  default void setVoltage(double voltage) {}
}
