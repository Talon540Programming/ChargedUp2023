package frc.robot.arm.rotation;

import frc.lib.logging.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface ArmRotationIO extends LoggedIO<ArmRotationIO.ArmRotationIOInputs> {
  @AutoLog
  public class ArmRotationIOInputs {
    public double SupplyCurrentAmps;
    public double StatorCurrentAmps;
  }

  @Override
  default void updateInputs(ArmRotationIOInputs inputs) {}

  default void setVoltage(double voltage) {}
}
