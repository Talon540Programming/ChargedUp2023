package frc.robot.intake;

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
  }

  default void setVoltage(double voltage) {}

  default void setNeutralMode(Constants.NeutralMode mode) {}

  default boolean isStalled() {
    return false;
  }
}
