package frc.robot.arm.extension;

import frc.lib.logging.LoggedIO;
import org.littletonrobotics.junction.AutoLog;

public interface ArmExtensionIO extends LoggedIO<ArmExtensionIO.ArmExtensionIOInputs> {
  @AutoLog
  public class ArmExtensionIOInputs {
    public double DistanceTraveledMeters;
    public double VelocityMetersPerSecond;
  }

  @Override
  default void updateInputs(ArmExtensionIOInputs inputs) {}

  default void setVoltage(double voltage) {}

  default void setDistance(double distance) {}

  default void resetDistance() {}
}