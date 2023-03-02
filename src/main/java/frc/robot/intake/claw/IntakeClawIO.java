package frc.robot.intake.claw;

import frc.lib.logging.LoggedIO;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeClawIO extends LoggedIO<IntakeClawIO.IntakeClawInputs> {
  @AutoLog
  public class IntakeClawInputs {
    public double CurrentAmps;
    public double TemperatureCelsius;
  }

  @Override
  default void updateInputs(IntakeClawInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void setNeutralMode(Constants.NeutralMode mode) {}
}
